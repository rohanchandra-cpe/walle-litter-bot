#include "encoders.h"

#include "hardware/gpio.h"
#include "hardware/sync.h"

// The motors are mirror-mounted, so their raw counts have opposite signs. On the
// course robot the RIGHT encoder counts DOWN when the robot drives forward, so it
// is flipped and both wheels count up for forward motion (tested on the robot).
// If the bench test in the guide shows a wheel counting the wrong way, flip its sign.
#define LEFT_SIGN (+1)
#define RIGHT_SIGN (-1)

// Quadrature decoder lookup table.
// Index = 4 * previous_state + current_state, where state = 2*A + B.
// +1 / -1 = one step forward / backward, 0 = no change or an impossible jump.
static const int8_t QDEC[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0,
};

typedef struct {
    uint pin_a;
    uint pin_b;
    uint8_t state;
    volatile int32_t count;
} encoder_t;

static encoder_t left_enc = {ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN, 0, 0};
static encoder_t right_enc = {ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN, 0, 0};
static volatile uint32_t error_count = 0;

static uint8_t read_state(const encoder_t *enc) {
    return (uint8_t) ((gpio_get(enc->pin_a) << 1) | gpio_get(enc->pin_b));
}

static void update(encoder_t *enc) {
    uint8_t now = read_state(enc);
    uint8_t index = (uint8_t) ((enc->state << 2) | now);
    // Index 3, 6, 9, 12: both A and B changed at once, so an edge was missed
    if (now != enc->state && QDEC[index] == 0) {
        error_count++;
    }
    enc->count += QDEC[index];
    enc->state = now;
}

// Runs on every rising AND falling edge of all four encoder pins (x4 decoding)
static void encoder_irq(uint gpio, uint32_t events) {
    (void) events;
    if (gpio == ENCODER_LEFT_A_PIN || gpio == ENCODER_LEFT_B_PIN) {
        update(&left_enc);
    } else if (gpio == ENCODER_RIGHT_A_PIN || gpio == ENCODER_RIGHT_B_PIN) {
        update(&right_enc);
    }
}

static void setup_pin(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
}

void encoders_init(void) {
    const uint pins[] = {ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN,
                         ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN};
    for (int i = 0; i < 4; i++) {
        setup_pin(pins[i]);
    }
    left_enc.state = read_state(&left_enc);
    right_enc.state = read_state(&right_enc);

    const uint32_t edges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    gpio_set_irq_enabled_with_callback(pins[0], edges, true, &encoder_irq);
    for (int i = 1; i < 4; i++) {
        gpio_set_irq_enabled(pins[i], edges, true);
    }
}

void encoders_read(int32_t *left, int32_t *right) {
    // Pause interrupts briefly so both counts come from the same instant
    uint32_t irq_state = save_and_disable_interrupts();
    int32_t l = left_enc.count;
    int32_t r = right_enc.count;
    restore_interrupts(irq_state);
    *left = LEFT_SIGN * l;
    *right = RIGHT_SIGN * r;
}

uint32_t encoders_error_count(void) {
    return error_count;
}
