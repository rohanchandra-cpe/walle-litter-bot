#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include <stdint.h>

enum {
    DRIVE_STATUS_STOPPED = 0,
    DRIVE_STATUS_DRIVING = 1,
    DRIVE_STATUS_TIMEOUT_STOP = 2,
};

void drive_control_init(void);

// Apply a velocity command: linear in m/s (forward is positive) and angular
// in rad/s (counterclockwise, a left turn, is positive). Returns the status.
int32_t drive_control_command(float linear, float angular, uint32_t now_ms);

// Call often. Stops the wheels if no command arrived for DRIVE_TIMEOUT_MS.
int32_t drive_control_update(uint32_t now_ms);

void drive_control_stop(void);

#endif
