#include "ultrasonic.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

// The echo pin gets its own raw interrupt handler, so the encoders keep the
// default GPIO callback (the Pico SDK allows only one of those).
typedef enum { IDLE, WAITING } ping_state_t;

static volatile ping_state_t state = IDLE;
static volatile bool echo_started;
static volatile bool echo_finished;
static volatile uint32_t rise_us;
static volatile uint32_t echo_us;
static uint32_t trigger_us;
static uint32_t last_trigger_ms;
static int32_t latest_seq;
static int32_t latest_echo = ULTRASONIC_NO_PING_YET;

static void echo_irq(void) {
    uint32_t events = gpio_get_irq_event_mask(ULTRASONIC_ECHO_PIN);
    if (events == 0) {
        return;
    }
    gpio_acknowledge_irq(ULTRASONIC_ECHO_PIN, events);
    if (state != WAITING) {
        return;   // the end of an echo that already timed out
    }
    uint32_t now = time_us_32();
    if (events & GPIO_IRQ_EDGE_RISE) {
        rise_us = now;
        echo_started = true;
    }
    if ((events & GPIO_IRQ_EDGE_FALL) && echo_started) {
        echo_us = now - rise_us;
        echo_finished = true;
    }
}

void ultrasonic_init(void) {
    gpio_init(ULTRASONIC_TRIG_PIN);
    gpio_set_dir(ULTRASONIC_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRASONIC_TRIG_PIN, 0);

    gpio_init(ULTRASONIC_ECHO_PIN);
    gpio_set_dir(ULTRASONIC_ECHO_PIN, GPIO_IN);
    // Holds the pin low if the sensor is unplugged; negligible next to the divider
    gpio_pull_down(ULTRASONIC_ECHO_PIN);

    gpio_add_raw_irq_handler(ULTRASONIC_ECHO_PIN, echo_irq);
    gpio_set_irq_enabled(ULTRASONIC_ECHO_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
    last_trigger_ms = to_ms_since_boot(get_absolute_time());
}

static void finish(int32_t echo) {
    uint32_t irq_state = save_and_disable_interrupts();
    latest_echo = echo;
    latest_seq++;
    state = IDLE;
    restore_interrupts(irq_state);
}

void ultrasonic_update(void) {
    if (state == WAITING) {
        if (echo_finished) {
            finish((int32_t) echo_us);
        } else if (time_us_32() - trigger_us > ULTRASONIC_TIMEOUT_US) {
            finish(echo_started ? ULTRASONIC_ECHO_TIMEOUT : ULTRASONIC_NO_RESPONSE);
        }
        return;
    }

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (now_ms - last_trigger_ms < ULTRASONIC_PERIOD_MS) {
        return;
    }
    last_trigger_ms = now_ms;
    echo_started = false;
    echo_finished = false;
    state = WAITING;
    trigger_us = time_us_32();
    gpio_put(ULTRASONIC_TRIG_PIN, 1);
    busy_wait_us_32(10);   // the HC-SR04 needs a trigger pulse of at least 10 us
    gpio_put(ULTRASONIC_TRIG_PIN, 0);
}

void ultrasonic_latest(int32_t *seq, int32_t *echo) {
    uint32_t irq_state = save_and_disable_interrupts();
    *seq = latest_seq;
    *echo = latest_echo;
    restore_interrupts(irq_state);
}
