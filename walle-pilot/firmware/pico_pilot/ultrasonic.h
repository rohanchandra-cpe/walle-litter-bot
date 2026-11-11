#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

// HC-SR04 wiring. ECHO is a 5 V output: it must be stepped down before it reaches
// GP16, because Pico pins are 3.3 V only.
#define ULTRASONIC_TRIG_PIN 17
#define ULTRASONIC_ECHO_PIN 16

// One ping every 60 ms (the HC-SR04 datasheet's minimum cycle). A ping whose echo
// has not ended 30 ms after the trigger is a timeout (4 m is about 23 ms).
#define ULTRASONIC_PERIOD_MS 60
#define ULTRASONIC_TIMEOUT_US 30000

// Result codes for a ping that has no echo time
#define ULTRASONIC_NO_PING_YET 0      // no ping has finished since power-on
#define ULTRASONIC_ECHO_TIMEOUT (-1)  // echo started but did not end within 30 ms
#define ULTRASONIC_NO_RESPONSE (-2)   // echo never started: sensor missing or unpowered

// Set up the pins and the echo interrupt. Call after encoders_init().
void ultrasonic_init(void);

// Call often from the main loop: triggers a new ping every 60 ms and finishes
// the current one. Never blocks for more than the 10 us trigger pulse.
void ultrasonic_update(void);

// The most recent finished ping. seq goes up by one per finished ping, so a
// reader can tell new pings from repeats. echo is the echo time in microseconds
// (> 0) or one of the result codes above.
void ultrasonic_latest(int32_t *seq, int32_t *echo);

#endif
