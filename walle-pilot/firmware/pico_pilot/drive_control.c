#include "drive_control.h"

#include "tb6612.h"

// ===== Calibrate these for your robot =====
// Distance between the centers of the two drive wheels, in meters.
// Course robot: 5 in center to center (4 in between the inside edges of the
// wheels, plus half of each 1 in wide wheel).
#define WHEEL_SEPARATION_M 0.127f
// Wheel speed at full PWM, in m/s. TT 1:90 motors: about 0.38 m/s with no load
// (113 RPM). With encoders you can measure it (PID guide, Step 6).
#define MAX_WHEEL_SPEED_MPS 0.40f
// Smallest PWM fraction that turns a wheel under load. Lower commands are
// raised to at least this value so the motors do not stall.
#define MIN_DUTY 0.40f

// Same limits as the Gazebo robot's DiffDrive plugin
#define MAX_LINEAR_MPS 0.30f
#define MAX_ANGULAR_RPS 1.50f

// Stop if no command arrives for this long (lost Wi-Fi, crashed agent, ...)
#define DRIVE_TIMEOUT_MS 500

static int32_t status = DRIVE_STATUS_STOPPED;
static uint32_t last_command_ms;

static float clamp(float value, float limit) {
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

// Convert a wheel speed in m/s to a motor command from -1.0 to 1.0.
static float wheel_duty(float wheel_speed) {
    float fraction = clamp(wheel_speed / MAX_WHEEL_SPEED_MPS, 1.0f);
    if (fraction > -0.01f && fraction < 0.01f) {
        return 0.0f;
    }
    float magnitude = fraction < 0.0f ? -fraction : fraction;
    float duty = MIN_DUTY + (1.0f - MIN_DUTY) * magnitude;
    return fraction < 0.0f ? -duty : duty;
}

void drive_control_init(void) {
    tb6612_stop();
    status = DRIVE_STATUS_STOPPED;
    last_command_ms = 0;
}

int32_t drive_control_command(float linear, float angular, uint32_t now_ms) {
    linear = clamp(linear, MAX_LINEAR_MPS);
    angular = clamp(angular, MAX_ANGULAR_RPS);
    last_command_ms = now_ms;

    // Differential drive: the right wheel goes faster for a left turn.
    float left = linear - angular * WHEEL_SEPARATION_M / 2.0f;
    float right = linear + angular * WHEEL_SEPARATION_M / 2.0f;
    float left_duty = wheel_duty(left);
    float right_duty = wheel_duty(right);

    tb6612_set(left_duty, right_duty);
    status = (left_duty == 0.0f && right_duty == 0.0f) ? DRIVE_STATUS_STOPPED
                                                        : DRIVE_STATUS_DRIVING;
    return status;
}

int32_t drive_control_update(uint32_t now_ms) {
    if (status == DRIVE_STATUS_DRIVING &&
        now_ms - last_command_ms > DRIVE_TIMEOUT_MS) {
        tb6612_stop();
        status = DRIVE_STATUS_TIMEOUT_STOP;
    }
    return status;
}

void drive_control_stop(void) {
    tb6612_stop();
    status = DRIVE_STATUS_STOPPED;
}
