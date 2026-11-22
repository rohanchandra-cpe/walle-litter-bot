#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

// Camera tilt servo (FITEC FS5106B, WALL-E Phase 4). Signal on GP22 (PWM slice 3, channel
// A; the motors use slices 1 and 6). Power it from its own 5-6 V supply (a buck converter or
// UBEC off the 9 V motor supply), NOT from 9 V and not from the camera's 5 V rail; tie its
// ground to the Pico's.
//
// The firmware knows only pulse widths. Degrees, the level pulse and the tilt limits are
// calibrated on the host (walle-app/config/servo.yaml), so recalibrating needs no reflash.
// These limits are a hard clamp that no command can get past.
#define SERVO_PIN 22
#define SERVO_MIN_US 600
#define SERVO_MAX_US 2400
#define SERVO_BOOT_US 1500          // where the servo goes at power-on (about level)
#define SERVO_SLEW_US_PER_S 1000    // gentle moves: smaller current spikes, less shake

// Start 50 Hz PWM at SERVO_BOOT_US.
void servo_init(void);

// Ask for a new pulse width (clamped to SERVO_MIN_US..SERVO_MAX_US). The servo moves
// there at SERVO_SLEW_US_PER_S, driven by servo_update().
void servo_set_us(int32_t pulse_us);

// Call often from the main loop: steps the output toward the target.
void servo_update(uint32_t now_ms);

// The pulse width being output now, and the target it is moving to (microseconds).
int32_t servo_current_us(void);
int32_t servo_target_us(void);

#endif
