#include "bno055.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#define BNO055_ADDRESS 0x28
#define BNO055_CHIP_ID_REG 0x00
#define BNO055_CHIP_ID 0xA0
#define BNO055_PAGE_ID_REG 0x07
// Data registers are consecutive, so one burst read from GYR_DATA (0x14)
// through the end of LIA_DATA (0x2D) gets everything we need:
//   0x14-0x19 gyroscope x, y, z
//   0x1A-0x1F Euler heading, roll, pitch (not used)
//   0x20-0x27 quaternion w, x, y, z
//   0x28-0x2D linear acceleration x, y, z
#define BNO055_BURST_START_REG 0x14
#define BNO055_BURST_LENGTH 26
#define BNO055_GYRO_OFFSET 0
#define BNO055_QUAT_OFFSET 12
#define BNO055_LIN_ACCEL_OFFSET 20
#define BNO055_CALIB_STAT_REG 0x35
#define BNO055_UNIT_SEL_REG 0x3B
#define BNO055_OPR_MODE_REG 0x3D
#define BNO055_POWER_MODE_REG 0x3E
#define BNO055_CONFIG_MODE 0x00
// IMU mode fuses the gyroscope and accelerometer only. Orientation starts at
// the power-on direction and is not disturbed by the motors' magnetic fields.
#define BNO055_IMU_MODE 0x08
// UNIT_SEL: accelerometer in m/s^2, gyroscope in rad/s (bit 1), Euler in degrees
#define BNO055_UNITS_GYRO_RAD_S 0x02

// Scale factors from the BNO055 datasheet (section 3.6.4)
#define QUAT_LSB_PER_UNIT 16384.0f      // 2^14
#define GYRO_LSB_PER_RAD_S 900.0f
#define ACCEL_LSB_PER_M_S2 100.0f

#define BNO055_SDA_PIN 0
#define BNO055_SCL_PIN 1
#define BNO055_I2C_TIMEOUT_US 10000

static bool write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_write_timeout_us(
               i2c0,
               BNO055_ADDRESS,
               data,
               sizeof(data),
               false,
               BNO055_I2C_TIMEOUT_US) == (int) sizeof(data);
}

static bool read_registers(uint8_t reg, uint8_t *data, size_t length) {
    if (i2c_write_timeout_us(
            i2c0,
            BNO055_ADDRESS,
            &reg,
            1,
            true,
            BNO055_I2C_TIMEOUT_US) != 1) {
        return false;
    }
    return i2c_read_timeout_us(
               i2c0,
               BNO055_ADDRESS,
               data,
               length,
               false,
               BNO055_I2C_TIMEOUT_US) == (int) length;
}

static int16_t int16_at(const uint8_t *data, size_t offset) {
    return (int16_t) (((uint16_t) data[offset + 1] << 8) | data[offset]);
}

bool bno055_init(void) {
    i2c_init(i2c0, 100000);
    gpio_set_function(BNO055_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(BNO055_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(BNO055_SDA_PIN);
    gpio_pull_up(BNO055_SCL_PIN);
    sleep_ms(700);

    uint8_t chip_id = 0;
    if (!read_registers(BNO055_CHIP_ID_REG, &chip_id, 1) ||
        chip_id != BNO055_CHIP_ID) {
        printf("BNO055 not found; chip ID was 0x%02X\n", chip_id);
        return false;
    }

    if (!write_register(BNO055_OPR_MODE_REG, BNO055_CONFIG_MODE)) {
        return false;
    }
    sleep_ms(25);

    if (!write_register(BNO055_PAGE_ID_REG, 0) ||
        !write_register(BNO055_POWER_MODE_REG, 0) ||
        !write_register(BNO055_UNIT_SEL_REG, BNO055_UNITS_GYRO_RAD_S) ||
        !write_register(BNO055_OPR_MODE_REG, BNO055_IMU_MODE)) {
        return false;
    }
    sleep_ms(20);
    return true;
}

bool bno055_read(bno055_reading_t *reading) {
    if (reading == NULL) {
        return false;
    }

    uint8_t data[BNO055_BURST_LENGTH];
    if (!read_registers(BNO055_BURST_START_REG, data, sizeof(data))) {
        return false;
    }

    reading->gx = int16_at(data, BNO055_GYRO_OFFSET + 0) / GYRO_LSB_PER_RAD_S;
    reading->gy = int16_at(data, BNO055_GYRO_OFFSET + 2) / GYRO_LSB_PER_RAD_S;
    reading->gz = int16_at(data, BNO055_GYRO_OFFSET + 4) / GYRO_LSB_PER_RAD_S;

    reading->qw = int16_at(data, BNO055_QUAT_OFFSET + 0) / QUAT_LSB_PER_UNIT;
    reading->qx = int16_at(data, BNO055_QUAT_OFFSET + 2) / QUAT_LSB_PER_UNIT;
    reading->qy = int16_at(data, BNO055_QUAT_OFFSET + 4) / QUAT_LSB_PER_UNIT;
    reading->qz = int16_at(data, BNO055_QUAT_OFFSET + 6) / QUAT_LSB_PER_UNIT;

    reading->ax = int16_at(data, BNO055_LIN_ACCEL_OFFSET + 0) / ACCEL_LSB_PER_M_S2;
    reading->ay = int16_at(data, BNO055_LIN_ACCEL_OFFSET + 2) / ACCEL_LSB_PER_M_S2;
    reading->az = int16_at(data, BNO055_LIN_ACCEL_OFFSET + 4) / ACCEL_LSB_PER_M_S2;

    // Right after power-on the chip reports an all-zero quaternion. A valid
    // orientation has length 1; normalize away the small rounding error.
    float norm = sqrtf(reading->qw * reading->qw + reading->qx * reading->qx +
                       reading->qy * reading->qy + reading->qz * reading->qz);
    if (norm < 0.9f || norm > 1.1f) {
        return false;
    }
    reading->qw /= norm;
    reading->qx /= norm;
    reading->qy /= norm;
    reading->qz /= norm;
    return true;
}

bool bno055_read_calibration(bno055_calibration_t *calibration) {
    if (calibration == NULL) {
        return false;
    }

    uint8_t status = 0;
    if (!read_registers(BNO055_CALIB_STAT_REG, &status, 1)) {
        return false;
    }
    calibration->system = (status >> 6) & 0x03;
    calibration->gyro = (status >> 4) & 0x03;
    calibration->accel = (status >> 2) & 0x03;
    calibration->mag = status & 0x03;
    return true;
}
