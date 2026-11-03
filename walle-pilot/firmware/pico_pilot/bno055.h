#ifndef BNO055_H
#define BNO055_H

#include <stdbool.h>
#include <stdint.h>

// One reading from the BNO055 in IMU mode (gyroscope + accelerometer fusion,
// magnetometer off). All values use the sensor's own right-handed axes:
// X and Y are printed on the board, Z points up out of the chip.
typedef struct {
    // Orientation as a unit quaternion. In IMU mode it is relative to the
    // direction the sensor faced at power-on; a left turn (counterclockwise
    // seen from above) increases the yaw.
    float qw, qx, qy, qz;
    // Angular velocity (gyroscope) in rad/s, counterclockwise positive.
    float gx, gy, gz;
    // Linear acceleration (gravity already removed by the chip) in m/s^2.
    float ax, ay, az;
} bno055_reading_t;

// Calibration status, each 0 (not calibrated) to 3 (fully calibrated).
typedef struct {
    uint8_t system, gyro, accel, mag;
} bno055_calibration_t;

bool bno055_init(void);

// Reads orientation, angular velocity, and linear acceleration in one burst,
// so all three describe the same instant. Returns false on an I2C error or
// if the chip has not produced a valid orientation yet.
bool bno055_read(bno055_reading_t *reading);

bool bno055_read_calibration(bno055_calibration_t *calibration);

#endif
