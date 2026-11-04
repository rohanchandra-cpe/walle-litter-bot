#include "tb6612.h"

#include <stdbool.h>
#include <stddef.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define MOTOR_PWMA_PIN 2
#define MOTOR_AIN1_PIN 4
#define MOTOR_AIN2_PIN 3
#define MOTOR_PWMB_PIN 12
#define MOTOR_BIN1_PIN 10
#define MOTOR_BIN2_PIN 11
#define MOTOR_STBY_PIN 5

#define MOTOR_PWM_TOP 999

void tb6612_init(void) {
    const uint pins[] = {
        MOTOR_AIN1_PIN,
        MOTOR_AIN2_PIN,
        MOTOR_BIN1_PIN,
        MOTOR_BIN2_PIN,
        MOTOR_STBY_PIN,
    };
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_OUT);
        gpio_put(pins[i], false);
    }

    gpio_set_function(MOTOR_PWMA_PIN, GPIO_FUNC_PWM);
    gpio_set_function(MOTOR_PWMB_PIN, GPIO_FUNC_PWM);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, MOTOR_PWM_TOP);
    pwm_init(pwm_gpio_to_slice_num(MOTOR_PWMA_PIN), &config, true);
    pwm_init(pwm_gpio_to_slice_num(MOTOR_PWMB_PIN), &config, true);
    pwm_set_gpio_level(MOTOR_PWMA_PIN, 0);
    pwm_set_gpio_level(MOTOR_PWMB_PIN, 0);
}

void tb6612_stop(void) {
    pwm_set_gpio_level(MOTOR_PWMA_PIN, 0);
    pwm_set_gpio_level(MOTOR_PWMB_PIN, 0);
    gpio_put(MOTOR_STBY_PIN, false);
}

static uint16_t pwm_level(float speed) {
    float magnitude = speed < 0.0f ? -speed : speed;
    if (magnitude > 1.0f) {
        magnitude = 1.0f;
    }
    return (uint16_t) (magnitude * MOTOR_PWM_TOP + 0.5f);
}

void tb6612_set(float left, float right) {
    if (left == 0.0f && right == 0.0f) {
        tb6612_stop();
        return;
    }

    // The motors are mirror-mounted, so the left motor (A) is reversed to make
    // both wheels roll the same way when wired M+ to AO1/BO1 and M- to AO2/BO2.
    bool left_forward = left >= 0.0f;
    bool right_forward = right >= 0.0f;
    gpio_put(MOTOR_AIN1_PIN, !left_forward);
    gpio_put(MOTOR_AIN2_PIN, left_forward);
    gpio_put(MOTOR_BIN1_PIN, right_forward);
    gpio_put(MOTOR_BIN2_PIN, !right_forward);
    pwm_set_gpio_level(MOTOR_PWMA_PIN, pwm_level(left));
    pwm_set_gpio_level(MOTOR_PWMB_PIN, pwm_level(right));
    gpio_put(MOTOR_STBY_PIN, true);
}
