// WALL-E pilot firmware: Module 8 sensor fusion firmware + an HC-SR04 range sensor.
//
// Publishes   /pico/wheel_ticks   std_msgs/Int32MultiArray  50 Hz:
//                                 [left, right, ping_seq, echo_us]
//                                 left/right: total encoder counts
//                                 ping_seq: +1 per finished ultrasonic ping (every 60 ms)
//                                 echo_us: that ping's echo time in us, or -1 echo
//                                          timeout, -2 no response, 0 no ping yet
//                                 The ping rides on this message because the micro-ROS
//                                 library allows only 3 publishers; nodes that read
//                                 just [left, right] keep working.
//             /pico/imu/data      sensor_msgs/Imu           orientation, gyro, accel, 50 Hz
//             /pico/drive/status  std_msgs/Int32            0 stopped, 1 driving, 2 timeout
// Subscribes  /pico/cmd_vel       geometry_msgs/Twist       drive command
//             /pico/imu/std       std_msgs/Float32MultiArray
//                                 [orientation_deg, gyro_rad_s, accel_m_s2] uncertainty

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "geometry_msgs/msg/twist.h"
#include "rcl/error_handling.h"
#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "sensor_msgs/msg/imu.h"
#include "std_msgs/msg/float32_multi_array.h"
#include "std_msgs/msg/int32.h"
#include "std_msgs/msg/int32_multi_array.h"

#include "bno055.h"
#include "drive_control.h"
#include "encoders.h"
#include "picow_udp_transport.h"
#include "tb6612.h"
#include "ultrasonic.h"
#include "wifi_config.h"

// ===== Measurement uncertainty (1 standard deviation) =====
// The EKF uses these to decide how much to trust the IMU. They are starting
// values: tune them live with /pico/imu/std, then copy your final values here.
#define DEFAULT_ORIENTATION_STD_DEG 2.0f
#define DEFAULT_GYRO_STD_RAD_S 0.02f
#define DEFAULT_ACCEL_STD_M_S2 0.30f
// While the gyroscope is not fully calibrated, report 10x the uncertainty
// (100x the variance) so the EKF trusts the IMU much less.
#define UNCALIBRATED_VARIANCE_SCALE 100.0
// Re-synchronize the Pico clock with the agent this often.
#define TIME_SYNC_PERIOD_MS 60000

#define PUBLISH_PERIOD_MS 20   // 50 Hz for wheel ticks and IMU

static rcl_publisher_t wheel_ticks_publisher;
static rcl_publisher_t imu_publisher;
static rcl_publisher_t drive_status_publisher;
static geometry_msgs__msg__Twist cmd_vel_message;
static std_msgs__msg__Int32MultiArray wheel_ticks_message;
static int32_t wheel_ticks_buffer[4];
static sensor_msgs__msg__Imu imu_message;
static char imu_frame_id[] = "imu_link";
static std_msgs__msg__Float32MultiArray imu_std_message;
static float imu_std_buffer[3];
static std_msgs__msg__Int32 drive_status_message;
static uint32_t commands_received;

static float orientation_std_deg = DEFAULT_ORIENTATION_STD_DEG;
static float gyro_std_rad_s = DEFAULT_GYRO_STD_RAD_S;
static float accel_std_m_s2 = DEFAULT_ACCEL_STD_M_S2;

static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static void publish_drive_status(int32_t status) {
    drive_status_message.data = status;
    if (rcl_publish(&drive_status_publisher, &drive_status_message, NULL) !=
        RCL_RET_OK) {
        printf("Failed to publish drive status.\n");
        rcl_reset_error();
    }
}

static void cmd_vel_callback(const void *incoming) {
    const geometry_msgs__msg__Twist *command = incoming;
    drive_control_command(
        (float) command->linear.x, (float) command->angular.z, now_ms());
    commands_received++;
}

// /pico/imu/std: [orientation_deg, gyro_rad_s, accel_m_s2]. A value of 0 or
// less keeps the current setting, so you can change just one of them.
static void imu_std_callback(const void *incoming) {
    const std_msgs__msg__Float32MultiArray *msg = incoming;
    const float *v = msg->data.data;
    size_t n = msg->data.size;
    if (n > 0 && v[0] > 0.0f) {
        orientation_std_deg = v[0];
    }
    if (n > 1 && v[1] > 0.0f) {
        gyro_std_rad_s = v[1];
    }
    if (n > 2 && v[2] > 0.0f) {
        accel_std_m_s2 = v[2];
    }
    printf("IMU uncertainty: orientation %.2f deg, gyro %.4f rad/s, accel %.3f m/s^2\n",
           orientation_std_deg, gyro_std_rad_s, accel_std_m_s2);
}

static void set_diagonal(double covariance[9], double variance) {
    memset(covariance, 0, 9 * sizeof(double));
    covariance[0] = variance;
    covariance[4] = variance;
    covariance[8] = variance;
}

static void fill_imu_message(const bno055_reading_t *r, bool gyro_calibrated) {
    int64_t nanos = rmw_uros_epoch_nanos();
    imu_message.header.stamp.sec = (int32_t) (nanos / 1000000000LL);
    imu_message.header.stamp.nanosec = (uint32_t) (nanos % 1000000000LL);

    imu_message.orientation.w = r->qw;
    imu_message.orientation.x = r->qx;
    imu_message.orientation.y = r->qy;
    imu_message.orientation.z = r->qz;
    imu_message.angular_velocity.x = r->gx;
    imu_message.angular_velocity.y = r->gy;
    imu_message.angular_velocity.z = r->gz;
    imu_message.linear_acceleration.x = r->ax;
    imu_message.linear_acceleration.y = r->ay;
    imu_message.linear_acceleration.z = r->az;

    double scale = gyro_calibrated ? 1.0 : UNCALIBRATED_VARIANCE_SCALE;
    double orientation_std_rad = orientation_std_deg * M_PI / 180.0;
    set_diagonal(imu_message.orientation_covariance,
                 scale * orientation_std_rad * orientation_std_rad);
    set_diagonal(imu_message.angular_velocity_covariance,
                 scale * gyro_std_rad_s * gyro_std_rad_s);
    set_diagonal(imu_message.linear_acceleration_covariance,
                 accel_std_m_s2 * accel_std_m_s2);
}

static float yaw_degrees(const bno055_reading_t *r) {
    float yaw = atan2f(2.0f * (r->qw * r->qz + r->qx * r->qy),
                       1.0f - 2.0f * (r->qy * r->qy + r->qz * r->qz));
    return yaw * 180.0f / (float) M_PI;
}

static bool check_rcl(rcl_ret_t result, const char *operation) {
    if (result == RCL_RET_OK) {
        return true;
    }
    printf("%s failed: %d\n", operation, (int) result);
    return false;
}

int main(void) {
    stdio_init_all();
    sleep_ms(1500);
    tb6612_init();
    drive_control_init();
    encoders_init();
    printf("Encoders ready on GP%d/GP%d (left) and GP%d/GP%d (right).\n",
           ENCODER_LEFT_A_PIN, ENCODER_LEFT_B_PIN, ENCODER_RIGHT_A_PIN, ENCODER_RIGHT_B_PIN);
    ultrasonic_init();
    printf("HC-SR04 on TRIG GP%d / ECHO GP%d, one ping every %d ms.\n",
           ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN, ULTRASONIC_PERIOD_MS);

    // micro-ROS needs all message memory set up in advance.
    wheel_ticks_message.data.data = wheel_ticks_buffer;
    wheel_ticks_message.data.size = 4;
    wheel_ticks_message.data.capacity = 4;
    wheel_ticks_message.layout.dim.data = NULL;
    wheel_ticks_message.layout.dim.size = 0;
    wheel_ticks_message.layout.dim.capacity = 0;
    wheel_ticks_message.layout.data_offset = 0;

    imu_std_message.data.data = imu_std_buffer;
    imu_std_message.data.size = 0;
    imu_std_message.data.capacity = 3;
    imu_std_message.layout.dim.data = NULL;
    imu_std_message.layout.dim.size = 0;
    imu_std_message.layout.dim.capacity = 0;
    imu_std_message.layout.data_offset = 0;

    imu_message.header.frame_id.data = imu_frame_id;
    imu_message.header.frame_id.size = strlen(imu_frame_id);
    imu_message.header.frame_id.capacity = sizeof(imu_frame_id);

    if (cyw43_arch_init()) {
        printf("Wi-Fi initialization failed.\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();
    printf("Connecting to %s...\n", WIFI_SSID);

    if (cyw43_arch_wifi_connect_timeout_ms(
            WIFI_SSID,
            WIFI_PASSWORD,
            CYW43_AUTH_WPA2_AES_PSK,
            30000)) {
        printf("Wi-Fi connection failed.\n");
        return 1;
    }

    uint8_t *ip_address = (uint8_t *) &cyw43_state.netif[0].ip_addr.addr;
    printf("Connected. IP address: %d.%d.%d.%d\n",
           ip_address[0], ip_address[1], ip_address[2], ip_address[3]);

    // The robot can still drive without the IMU, but it will not publish IMU data.
    bool imu_ready = bno055_init();
    if (imu_ready) {
        printf("BNO055 ready in IMU mode. Keep the robot still while the gyro calibrates.\n");
    } else {
        printf("BNO055 not found; /pico/imu/data will not be published.\n");
    }

    static picow_udp_transport_context_t transport_context = {
        .agent_ip = ROS_AGENT_IP,
        .agent_port = ROS_AGENT_PORT,
        .pcb = NULL,
        .receive_head = 0,
        .receive_tail = 0,
        .receive_count = 0,
    };
    if (rmw_uros_set_custom_transport(
            false,
            &transport_context,
            picow_udp_transport_open,
            picow_udp_transport_close,
            picow_udp_transport_write,
            picow_udp_transport_read) != RMW_RET_OK) {
        printf("Could not configure the micro-ROS UDP transport.\n");
        return 1;
    }

    printf("Pinging micro-ROS Agent at %s:%u...\n",
           ROS_AGENT_IP,
           (unsigned int) ROS_AGENT_PORT);
    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        printf("Waiting for micro-ROS Agent...\n");
        sleep_ms(1000);
    }
    printf("micro-ROS Agent connected.\n");

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_subscription_t cmd_vel_subscription = rcl_get_zero_initialized_subscription();
    rcl_subscription_t imu_std_subscription = rcl_get_zero_initialized_subscription();
    wheel_ticks_publisher = rcl_get_zero_initialized_publisher();
    imu_publisher = rcl_get_zero_initialized_publisher();
    drive_status_publisher = rcl_get_zero_initialized_publisher();
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

    // 3 publishers and 2 subscriptions: exactly what the Module 5 micro-ROS
    // library allows, so no library rebuild is needed.
    if (!check_rcl(
            rclc_support_init(&support, 0, NULL, &allocator),
            "rclc_support_init") ||
        !check_rcl(
            rclc_node_init_default(&node, "pico_pilot", "", &support),
            "rclc_node_init_default") ||
        !check_rcl(
            rclc_publisher_init_best_effort(
                &wheel_ticks_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
                "/pico/wheel_ticks"),
            "wheel ticks publisher") ||
        !check_rcl(
            // Best effort, like all sensor streams: a lost reading is replaced
            // 20 ms later by a newer one, so waiting for a resend is pointless.
            rclc_publisher_init_best_effort(
                &imu_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                "/pico/imu/data"),
            "IMU publisher") ||
        !check_rcl(
            rclc_publisher_init_default(
                &drive_status_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
                "/pico/drive/status"),
            "drive status publisher") ||
        !check_rcl(
            rclc_subscription_init_best_effort(
                &cmd_vel_subscription,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
                "/pico/cmd_vel"),
            "cmd_vel subscription") ||
        !check_rcl(
            rclc_subscription_init_default(
                &imu_std_subscription,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                "/pico/imu/std"),
            "IMU uncertainty subscription") ||
        !check_rcl(
            rclc_executor_init(&executor, &support.context, 2, &allocator),
            "rclc_executor_init") ||
        !check_rcl(
            rclc_executor_add_subscription(
                &executor,
                &cmd_vel_subscription,
                &cmd_vel_message,
                cmd_vel_callback,
                ON_NEW_DATA),
            "cmd_vel executor subscription") ||
        !check_rcl(
            rclc_executor_add_subscription(
                &executor,
                &imu_std_subscription,
                &imu_std_message,
                imu_std_callback,
                ON_NEW_DATA),
            "IMU uncertainty executor subscription")) {
        drive_control_stop();
        return 1;
    }

    // Every IMU message carries the time it was measured. The Pico only knows
    // the time since it booted, so it asks the agent for the real clock time.
    while (!rmw_uros_epoch_synchronized()) {
        printf("Synchronizing clock with the micro-ROS Agent...\n");
        rmw_uros_sync_session(1000);
    }
    printf("Clock synchronized.\n");

    printf("micro-ROS node ready. Publishing /pico/wheel_ticks (with pings) and /pico/imu/data, "
           "listening on /pico/cmd_vel.\n");
    int32_t last_status = DRIVE_STATUS_STOPPED;
    bool gyro_was_calibrated = false;
    bno055_reading_t reading = {0};
    bool reading_valid = false;
    bno055_calibration_t calibration = {0};
    absolute_time_t next_publish = make_timeout_time_ms(PUBLISH_PERIOD_MS);
    absolute_time_t next_status_publish = make_timeout_time_ms(1000);
    absolute_time_t next_log = make_timeout_time_ms(1000);
    absolute_time_t next_time_sync = make_timeout_time_ms(TIME_SYNC_PERIOD_MS);

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
        ultrasonic_update();

        int32_t status = drive_control_update(now_ms());
        if (status != last_status || time_reached(next_status_publish)) {
            if (status == DRIVE_STATUS_TIMEOUT_STOP &&
                last_status != DRIVE_STATUS_TIMEOUT_STOP) {
                printf("No /pico/cmd_vel for 500 ms; wheels stopped.\n");
            }
            publish_drive_status(status);
            last_status = status;
            next_status_publish = make_timeout_time_ms(1000);
        }

        // 50 Hz: encoder counts and IMU data, read back to back
        if (time_reached(next_publish)) {
            encoders_read(&wheel_ticks_buffer[0], &wheel_ticks_buffer[1]);
            ultrasonic_latest(&wheel_ticks_buffer[2], &wheel_ticks_buffer[3]);
            if (rcl_publish(&wheel_ticks_publisher, &wheel_ticks_message, NULL) !=
                RCL_RET_OK) {
                rcl_reset_error();
            }

            if (imu_ready) {
                reading_valid = bno055_read(&reading);
                bool gyro_calibrated = calibration.gyro == 3;
                if (gyro_calibrated && !gyro_was_calibrated) {
                    printf("Gyroscope calibrated. Publishing /pico/imu/data.\n");
                    gyro_was_calibrated = true;
                }
                // Wait for the first full gyro calibration before publishing,
                // so the EKF never starts from an uncalibrated heading.
                if (reading_valid && gyro_was_calibrated) {
                    fill_imu_message(&reading, gyro_calibrated);
                    if (rcl_publish(&imu_publisher, &imu_message, NULL) != RCL_RET_OK) {
                        rcl_reset_error();
                    }
                }
            }

            next_publish = delayed_by_ms(next_publish, PUBLISH_PERIOD_MS);
            if (time_reached(next_publish)) {
                next_publish = make_timeout_time_ms(PUBLISH_PERIOD_MS);
            }
        }

        if (time_reached(next_time_sync)) {
            rmw_uros_sync_session(50);
            next_time_sync = make_timeout_time_ms(TIME_SYNC_PERIOD_MS);
        }

        if (time_reached(next_log)) {
            if (imu_ready) {
                bno055_read_calibration(&calibration);
            }
            int32_t left = 0, right = 0;
            encoders_read(&left, &right);
            int32_t ping_seq = 0, echo = 0;
            ultrasonic_latest(&ping_seq, &echo);
            printf("Ticks L %ld  R %ld  (errors %lu) | yaw %.1f deg  gyro z %+.3f rad/s | "
                   "cal sys %u gyro %u accel %u | drive %ld | %lu commands | "
                   "ping %ld echo %ld us\n",
                   (long) left, (long) right, (unsigned long) encoders_error_count(),
                   reading_valid ? yaw_degrees(&reading) : 0.0f,
                   reading_valid ? reading.gz : 0.0f,
                   calibration.system, calibration.gyro, calibration.accel,
                   (long) status, (unsigned long) commands_received,
                   (long) ping_seq, (long) echo);
            next_log = make_timeout_time_ms(1000);
        }
    }
}
