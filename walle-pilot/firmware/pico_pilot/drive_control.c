#include "drive_control.h"

#include <math.h>
#include <stdbool.h>

#include "encoders.h"
#include "tb6612.h"

// Closed-loop wheel speed control (WALL-E Phase 1, 2026-11-18). Before this, a
// command was turned straight into PWM with a 40 % floor, so any slow command ran
// at about 0.2 m/s and any turn at about 3 rad/s. Now each wheel's speed is
// measured from its encoder every CONTROL_PERIOD_MS and a PI loop (on top of a
// feed-forward guess) holds the commanded speed, so slow turns are really slow.
// Based on module 7's firmware/pico_pid/wheel_control.c, with fixed gains (the
// micro-ROS library has no free subscription for a gains topic). Mirrored in
// walle-sim/nodes/firmware_models.py (PidFirmware): change both together.

// ===== Robot geometry =====
// Distance between the centers of the two drive wheels, in meters.
#define WHEEL_SEPARATION_M 0.127f
#define WHEEL_DIAMETER_M 0.0635f
#define M_PER_COUNT (3.14159265f * WHEEL_DIAMETER_M / ENCODER_COUNTS_PER_REV)

// ===== Motor model (feed-forward) =====
// Wheel speed at full PWM, in m/s (pilot: 0.27 m/s at duty 0.625 -> 0.432).
#define MAX_WHEEL_SPEED_MPS 0.432f
// Smallest PWM fraction used. Any effort above 0.01 starts here, so the motor does
// not sit in its dead zone; the PI loop brings the average speed down from there.
#define MIN_DUTY 0.25f

// ===== PI speed loop =====
// Module 7's KP 0.1 / filter 0.5 made WALL-E lurch and stutter (run 20261119_015242). Retuned
// 2026-11-18 in walle-sim against a stiction model fitted to that run: KP 1.0 and a lighter
// filter back the motor off faster once a stuck wheel breaks free. Below about 0.1 m/s per
// wheel these motors stick and lurch whatever the gains: the route follower avoids it.
#define KP 1.0f                    // effort per (m/s) of speed error
#define KI 5.0f                    // effort per (m) of accumulated speed error
#define CONTROL_PERIOD_MS 20       // 50 Hz
#define SPEED_FILTER 0.3f          // 0..1: share of the previous speed kept (smoothing)
#define MAX_ACCEL_MPS2 0.5f        // setpoint ramp: smooth starts (stops are immediate)

// Same limits as the Gazebo robot's DiffDrive plugin
#define MAX_LINEAR_MPS 0.30f
#define MAX_ANGULAR_RPS 1.50f

// Stop if no command arrives for this long (lost Wi-Fi, crashed agent, ...)
#define DRIVE_TIMEOUT_MS 500

typedef struct {
    float target;       // from the last command (m/s)
    float setpoint;     // target after the acceleration ramp (m/s)
    float measured;     // filtered speed from the encoder (m/s)
    float integral;     // accumulated speed error (m)
    float effort;       // -1..1
    int32_t last_counts;
} wheel_t;

static wheel_t left_wheel, right_wheel;
static int32_t status = DRIVE_STATUS_STOPPED;
static uint32_t last_command_ms;
static uint32_t last_control_ms;
static bool have_counts = false;

static float clamp(float value, float limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

// Effort (-1..1) -> motor duty, skipping the dead zone where the motor does not turn.
static float effort_to_duty(float effort) {
    if (fabsf(effort) < 0.01f) {
        return 0.0f;
    }
    float duty = MIN_DUTY + (1.0f - MIN_DUTY) * fabsf(effort);
    return effort < 0.0f ? -duty : duty;
}

static void reset_wheel(wheel_t *wheel) {
    wheel->target = 0.0f;
    wheel->setpoint = 0.0f;
    wheel->integral = 0.0f;
    wheel->effort = 0.0f;
}

static void control_wheel(wheel_t *wheel, int32_t counts, float dt) {
    // Measure: counts since the last step -> speed, lightly filtered
    float raw = (float) (counts - wheel->last_counts) * M_PER_COUNT / dt;
    wheel->last_counts = counts;
    wheel->measured = SPEED_FILTER * wheel->measured + (1.0f - SPEED_FILTER) * raw;

    // Stop: motor off and integral cleared at once (a leftover integral would
    // drive the wheel backward while it waits to reach 0).
    if (wheel->target == 0.0f) {
        wheel->setpoint = 0.0f;
        wheel->integral = 0.0f;
        wheel->effort = 0.0f;
        return;
    }
    wheel->setpoint += clamp(wheel->target - wheel->setpoint, MAX_ACCEL_MPS2 * dt);

    float error = wheel->setpoint - wheel->measured;
    float integral = wheel->integral + error * dt;
    float u = wheel->setpoint / MAX_WHEEL_SPEED_MPS + KP * error + KI * integral;
    // Anti-windup: keep the new integral only while the output is not saturated
    if (u > 1.0f) {
        u = 1.0f;
    } else if (u < -1.0f) {
        u = -1.0f;
    } else {
        wheel->integral = integral;
    }
    wheel->effort = u;
}

void drive_control_init(void) {
    reset_wheel(&left_wheel);
    reset_wheel(&right_wheel);
    left_wheel.measured = right_wheel.measured = 0.0f;
    tb6612_stop();
    status = DRIVE_STATUS_STOPPED;
    last_command_ms = 0;
    have_counts = false;
}

int32_t drive_control_command(float linear, float angular, uint32_t now_ms) {
    linear = clamp(linear, MAX_LINEAR_MPS);
    angular = clamp(angular, MAX_ANGULAR_RPS);
    last_command_ms = now_ms;

    // Differential drive: the right wheel goes faster for a left turn.
    left_wheel.target = linear - angular * WHEEL_SEPARATION_M / 2.0f;
    right_wheel.target = linear + angular * WHEEL_SEPARATION_M / 2.0f;
    if (left_wheel.target == 0.0f && right_wheel.target == 0.0f) {
        drive_control_stop();          // a zero command stops at once, as before
    } else {
        status = DRIVE_STATUS_DRIVING;
    }
    return status;
}

int32_t drive_control_update(uint32_t now_ms) {
    // Watchdog: no fresh command -> stop the wheels now (never removed)
    if (status == DRIVE_STATUS_DRIVING && now_ms - last_command_ms > DRIVE_TIMEOUT_MS) {
        drive_control_stop();
        status = DRIVE_STATUS_TIMEOUT_STOP;
    }

    int32_t left_counts, right_counts;
    if (!have_counts) {
        encoders_read(&left_counts, &right_counts);
        left_wheel.last_counts = left_counts;
        right_wheel.last_counts = right_counts;
        last_control_ms = now_ms;
        have_counts = true;
        return status;
    }
    if (now_ms - last_control_ms < CONTROL_PERIOD_MS) {
        return status;
    }
    float dt = (float) (now_ms - last_control_ms) / 1000.0f;
    last_control_ms = now_ms;
    encoders_read(&left_counts, &right_counts);
    control_wheel(&left_wheel, left_counts, dt);
    control_wheel(&right_wheel, right_counts, dt);
    if (status == DRIVE_STATUS_DRIVING) {
        tb6612_set(effort_to_duty(left_wheel.effort), effort_to_duty(right_wheel.effort));
    }
    return status;
}

void drive_control_stop(void) {
    reset_wheel(&left_wheel);
    reset_wheel(&right_wheel);
    tb6612_stop();
    status = DRIVE_STATUS_STOPPED;
}
