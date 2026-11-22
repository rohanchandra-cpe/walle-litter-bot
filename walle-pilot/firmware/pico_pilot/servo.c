#include "servo.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

// The PWM counter runs at 1 MHz (system clock / its MHz: 150 on the Pico 2 W), so one count
// is 1 us and 20000 counts is 50 Hz.
#define SERVO_WRAP 19999

static int32_t current_us = SERVO_BOOT_US;
static int32_t target_us = SERVO_BOOT_US;
static uint32_t last_update_ms = 0;
static float carry_us = 0.0f;       // slew left over from short update intervals

static int32_t clamp_us(int32_t us) {
    if (us < SERVO_MIN_US) {
        return SERVO_MIN_US;
    }
    if (us > SERVO_MAX_US) {
        return SERVO_MAX_US;
    }
    return us;
}

static void output(int32_t us) {
    pwm_set_gpio_level(SERVO_PIN, (uint16_t) us);
}

void servo_init(void) {
    gpio_set_function(SERVO_PIN, GPIO_FUNC_PWM);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, (float) clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&config, SERVO_WRAP);
    pwm_init(pwm_gpio_to_slice_num(SERVO_PIN), &config, true);
    current_us = target_us = clamp_us(SERVO_BOOT_US);
    output(current_us);
}

void servo_set_us(int32_t pulse_us) {
    target_us = clamp_us(pulse_us);
}

void servo_update(uint32_t now_ms) {
    uint32_t dt = now_ms - last_update_ms;
    last_update_ms = now_ms;
    if (current_us == target_us) {
        carry_us = 0.0f;
        return;
    }
    if (dt > 100) {
        dt = 100;                   // first call, or a long stall: don't jump
    }
    carry_us += SERVO_SLEW_US_PER_S * (float) dt / 1000.0f;
    int32_t step = (int32_t) carry_us;
    if (step <= 0) {
        return;
    }
    carry_us -= (float) step;
    int32_t diff = target_us - current_us;
    if (diff > step) {
        current_us += step;
    } else if (diff < -step) {
        current_us -= step;
    } else {
        current_us = target_us;
    }
    output(current_us);
}

int32_t servo_current_us(void) {
    return current_us;
}

int32_t servo_target_us(void) {
    return target_us;
}
