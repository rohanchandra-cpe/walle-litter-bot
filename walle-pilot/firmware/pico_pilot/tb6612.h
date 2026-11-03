#ifndef TB6612_H
#define TB6612_H

void tb6612_init(void);
void tb6612_stop(void);

// Set each wheel's speed from -1.0 (full reverse) to 1.0 (full forward).
// 0.0 stops that wheel.
void tb6612_set(float left, float right);

#endif
