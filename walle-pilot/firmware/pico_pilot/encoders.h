#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdint.h>

// Course robot wiring
#define ENCODER_LEFT_A_PIN 18
#define ENCODER_LEFT_B_PIN 19
#define ENCODER_RIGHT_A_PIN 20
#define ENCODER_RIGHT_B_PIN 21

// Counts per wheel turn with x4 decoding: 12 PPR x 90 gear ratio x 4 edges
#define ENCODER_COUNTS_PER_REV 4320

// Start counting edges on both channels of both encoders (x4 decoding).
void encoders_init(void);

// Total counts since power-on. Both wheels count UP when the robot drives forward.
void encoders_read(int32_t *left, int32_t *right);

// Number of impossible transitions seen (both A and B changed at once).
// Should stay at or near 0; a growing number means missed edges or noise.
uint32_t encoders_error_count(void);

#endif
