// =============================================================================
// esp32_driver.ino — PortaMail motor/sensor firmware for ESP32-WROOM-32
//
// Drop-in replacement for teensy_driver.ino targeting the ESP32-WROOM-32.
// Publishes identical ROS 2 topics so the Pi-side stack (hardware.launch.py,
// EKF, SLAM Toolbox, Nav2) requires no changes.
//
// HARDWARE DIFFERENCES vs Teensy 4.0
// ----------------------------------------------------
//  • PWM   : Teensy uses analogWrite(); ESP32 uses the LEDC peripheral.
//  • USB   : Teensy has native USB → /dev/ttyACM0.
//            ESP32 uses an on-board CP2102 / CH340 USB-UART bridge
//            → appears as /dev/ttyUSBx (typically /dev/ttyUSB1 when the
//            LiDAR is already on /dev/ttyUSB0).
//            Launch with: mcu_port:=/dev/ttyUSB1  (see hardware.launch.py)
//  • ISRs  : must carry IRAM_ATTR to run from IRAM on ESP32.
//  • Atomic: portDISABLE_INTERRUPTS / portENABLE_INTERRUPTS instead of
//            noInterrupts / interrupts.
//  • I2C   : Wire.begin(SDA, SCL) — explicit pin numbers required on ESP32.
//
// VIABILITY NOTES
// ----------------------------------------------------
//  • micro-ROS Arduino library must be built for ROS 2 Jazzy + ESP32 target.
//    Use the micro_ros_arduino builder:
//    https://github.com/micro-ROS/micro_ros_arduino
//    Pre-built .zip for ESP32 / Jazzy may be available; otherwise build from
//    source following the micro-ROS Arduino README.
//  • Board in Arduino IDE: "ESP32 Dev Module" (esp32 by Espressif).
//  • ESP32 Arduino core v2.x: uses ledcSetup/ledcAttachPin (old API).
//    ESP32 Arduino core v3.x: uses ledcAttach(pin,freq,bits) + ledcWrite(pin,duty).
//    This sketch targets core v3.x. If you see ledcSetup errors, update the board.
//  • micro-ROS transport: set_microros_transports() (same macro as Teensy).
//    This resolves to the board-default transport. For ESP32 it targets Serial0.
//    If the library was built for WiFi-only, switch to serial via the build flags.
//  • Flash @ 921600 baud; micro-ROS runs @ 115200 baud on the same Serial port.
//  • GPIO12 must be LOW at boot (flash voltage strap) — avoid using it.
//  • GPIO6-11 are connected to internal flash — never use for GPIO.
//
// =============================================================================
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/range.h>

// =============================================================================
// HARDWARE PIN CONFIGURATION — ESP32-WROOM-32
//
// These are SUGGESTED assignments based on the WROOM-32 pinout image.
// Re-map to match your actual wiring before flashing.
//
// PINS TO AVOID:
//   GPIO 6-11  : connected to internal SPI flash — NEVER use
//   GPIO 12    : boot strapping (must be LOW at boot) — avoid
//   GPIO 0, 15 : boot strapping — use with caution
//   GPIO 34-39 : input-only, no internal pull-up/down
//
// MOTOR DRIVER — L298N
//   ENA = PWM speed control for left motor  (LEDC output)
//   ENB = PWM speed control for right motor (LEDC output)
//   IN1/IN2 = direction for left motor
//   IN3/IN4 = direction for right motor
//   NOTE: Remove the ENA/ENB jumpers on the L298N board; connect those
//         pins to the ESP32 GPIOs below for PWM speed control.
// =============================================================================

// Left motor
#define MOTOR_LEFT_INA   13   // Direction pin IN1
#define MOTOR_LEFT_INB   14   // Direction pin IN2
#define MOTOR_LEFT_ENA   16   // PWM speed → ENA (LEDC)

// Right motor
#define MOTOR_RIGHT_INA  27   // Direction pin IN3
#define MOTOR_RIGHT_INB  26   // Direction pin IN4
#define MOTOR_RIGHT_ENA  17   // PWM speed → ENB (LEDC)

// LEDC PWM parameters (ESP32 Arduino core v3.x pin-based API)
// ledcAttach(pin, freq, bits) replaces the old ledcSetup + ledcAttachPin.
// ledcWrite(pin, duty)        replaces the old ledcWrite(channel, duty).
#define LEDC_FREQ_HZ   5000   // 5 kHz — within L298N PWM range
#define LEDC_BITS      8      // 0-255 duty range, matches analogWrite

// FIT0186 Encoders — channel A on interrupt pin, B for direction
// Physical A/B and left/right assignment must be verified on the actual robot.
#define ENC_LEFT_A   32   // Interrupt (RISING) for tick count
#define ENC_LEFT_B   33   // Direction sense
#define ENC_RIGHT_A  25   // Interrupt (RISING) for tick count
#define ENC_RIGHT_B   4   // Direction sense

// BNO055 IMU — I2C address 0x28
// ESP32 default I2C: SDA=GPIO21, SCL=GPIO22
#define I2C_SDA  21
#define I2C_SCL  22

// HC-SR04 Ultrasonic ranger
// WARNING: HC-SR04 Echo outputs 5 V. The ESP32 is 3.3 V only.
//          Use a resistor voltage divider (e.g. 1 kΩ / 2 kΩ) or
//          a level shifter on the ECHO line — identical caveat as Teensy.
#define ULTRASONIC_TRIG   5   // OUTPUT — 10 µs HIGH pulse
#define ULTRASONIC_ECHO  18   // INPUT  — measure echo pulse width

#define LED_PIN  2   // Built-in LED on most ESP32 dev boards

// =============================================================================
// ROBOT CONSTANTS  (same as Teensy — calibrate on physical robot)
// =============================================================================
const float WHEEL_RADIUS  = 0.03556f;  // meters  (2.8" RC tires)
const float WHEEL_BASE    = 0.20f;     // meters  (*** MEASURE TRACK WIDTH ***)
const float TICKS_PER_REV = 720.0f;   // 8 PPR × 90:1  (*** CALIBRATE 1-m run ***)
const float MAX_SPEED_MPS = 0.89f;    // 2 MPH safety cap
const int   PWM_MIN       = 70;        // minimum PWM to overcome stiction
const int   PWM_MAX       = 255;

// =============================================================================
// BNO055
// =============================================================================
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
bool imu_ready = false;

// =============================================================================
// MICRO-ROS HANDLES
// =============================================================================
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t    odom_pub;
rcl_publisher_t    imu_pub;
rcl_publisher_t    range_pub;

geometry_msgs__msg__Twist   cmd_msg;
nav_msgs__msg__Odometry     odom_msg;
sensor_msgs__msg__Imu       imu_msg;
sensor_msgs__msg__Range     range_msg;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;

// =============================================================================
// ODOMETRY STATE
// =============================================================================
volatile long left_ticks  = 0;
volatile long right_ticks = 0;
unsigned long prev_odom_time = 0;
unsigned long last_cmd_time  = 0;
float x_pos = 0.0f;
float y_pos = 0.0f;
float theta  = 0.0f;

static char frame_odom[]       = "odom";
static char frame_base_link[]  = "base_link";
static char frame_imu_link[]   = "imu_link";
static char frame_ultrasonic[] = "ultrasonic_link";

// =============================================================================
// ENCODER ISRs — IRAM_ATTR keeps them in fast IRAM on ESP32
// =============================================================================
void IRAM_ATTR doEncoderLeft() {
  if (digitalRead(ENC_LEFT_B) == HIGH) left_ticks++;
  else                                  left_ticks--;
}
void IRAM_ATTR doEncoderRight() {
  if (digitalRead(ENC_RIGHT_B) == HIGH) right_ticks++;
  else                                   right_ticks--;
}

// =============================================================================
// HELPERS
// =============================================================================
float clamp(float val, float lo, float hi) {
  if (val > hi) return hi;
  if (val < lo) return lo;
  return val;
}

// ESP32 LEDC PWM (core v3.x) — replaces Teensy analogWrite().
// ena_pin is the actual GPIO number; ledcWrite takes pin, not channel.
void setMotor(int ena_pin, int in1, int in2, float speed) {
  int pwm_val = 0;
  if (fabsf(speed) > 0.01f) {
    float factor = fabsf(speed) / MAX_SPEED_MPS;
    pwm_val = PWM_MIN + (int)(factor * (PWM_MAX - PWM_MIN));
    if (pwm_val > PWM_MAX) pwm_val = PWM_MAX;
  }
  if      (speed >  0.01f) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else if (speed < -0.01f) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                     { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); pwm_val = 0; }
  ledcWrite(ena_pin, pwm_val);   // v3.x: pin-based, no channel argument
}

// =============================================================================
// ROS CALLBACK: /cmd_vel
// =============================================================================
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  last_cmd_time = millis();

  float linear  = clamp(msg->linear.x, -MAX_SPEED_MPS, MAX_SPEED_MPS);
  float angular = msg->angular.z;

  float left_spd  = clamp(linear - (angular * WHEEL_BASE / 2.0f), -MAX_SPEED_MPS, MAX_SPEED_MPS);
  float right_spd = clamp(linear + (angular * WHEEL_BASE / 2.0f), -MAX_SPEED_MPS, MAX_SPEED_MPS);

  setMotor(MOTOR_LEFT_ENA,  MOTOR_LEFT_INA,  MOTOR_LEFT_INB,  left_spd);
  setMotor(MOTOR_RIGHT_ENA, MOTOR_RIGHT_INA, MOTOR_RIGHT_INB, right_spd);
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);

  // Motor direction outputs
  pinMode(MOTOR_LEFT_INA,  OUTPUT); pinMode(MOTOR_LEFT_INB,  OUTPUT);
  pinMode(MOTOR_RIGHT_INA, OUTPUT); pinMode(MOTOR_RIGHT_INB, OUTPUT);

  // LEDC PWM setup (ESP32 Arduino core v3.x pin-based API)
  // ledcAttach(pin, freq, resolution_bits) — no channel numbers needed.
  ledcAttach(MOTOR_LEFT_ENA,  LEDC_FREQ_HZ, LEDC_BITS);
  ledcAttach(MOTOR_RIGHT_ENA, LEDC_FREQ_HZ, LEDC_BITS);
  ledcWrite(MOTOR_LEFT_ENA,  0);
  ledcWrite(MOTOR_RIGHT_ENA, 0);

  // Encoder inputs
  pinMode(ENC_LEFT_A,  INPUT_PULLUP); pinMode(ENC_LEFT_B,  INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP); pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  doEncoderLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), doEncoderRight, RISING);

  // Ultrasonic
  pinMode(ULTRASONIC_TRIG, OUTPUT); digitalWrite(ULTRASONIC_TRIG, LOW);
  pinMode(ULTRASONIC_ECHO, INPUT);

  // BNO055 — explicit SDA/SCL for ESP32
  Wire.begin(I2C_SDA, I2C_SCL);
  imu_ready = bno.begin();
  if (imu_ready) bno.setExtCrystalUse(true);

  // micro-ROS transport.
  // set_microros_transports() is the same macro used by the Teensy sketch.
  // For ESP32 it resolves to Serial (UART0 → CP2102 USB bridge → /dev/ttyUSBx).
  // If your library build only ships WiFi transport, you will see a compile error
  // here — in that case rebuild the micro-ROS Arduino library with SERIAL enabled:
  //   docker run --rm microros/micro_ros_setup:jazzy \
  //     ros2 run micro_ros_setup create_firmware_ws.sh generate_packages \
  //     --transport serial ...
  Serial.begin(115200);
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_driver", "", &support);
  rmw_uros_sync_session(1000);

  // Subscriber: /cmd_vel
  rclc_subscription_init_default(
    &cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

  // Publisher: /wheel/odom
  rclc_publisher_init_default(
    &odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "wheel/odom");

  // Publisher: /imu/data
  rclc_publisher_init_default(
    &imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");

  // Publisher: /ultrasonic/range
  rclc_publisher_init_default(
    &range_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "ultrasonic/range");

  // Static range message fields
  range_msg.radiation_type            = sensor_msgs__msg__Range__ULTRASOUND;
  range_msg.field_of_view             = 0.26f;
  range_msg.min_range                 = 0.02f;
  range_msg.max_range                 = 4.00f;
  range_msg.header.frame_id.data      = frame_ultrasonic;
  range_msg.header.frame_id.size      = strlen(frame_ultrasonic);
  range_msg.header.frame_id.capacity  = strlen(frame_ultrasonic) + 1;

  // Static IMU covariances
  imu_msg.header.frame_id.data     = frame_imu_link;
  imu_msg.header.frame_id.size     = strlen(frame_imu_link);
  imu_msg.header.frame_id.capacity = strlen(frame_imu_link) + 1;
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i]         = 0.0;
    imu_msg.angular_velocity_covariance[i]    = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }
  imu_msg.orientation_covariance[0]         = 0.0076;
  imu_msg.orientation_covariance[4]         = 0.0076;
  imu_msg.orientation_covariance[8]         = 0.0076;
  imu_msg.angular_velocity_covariance[0]    = 0.01;
  imu_msg.angular_velocity_covariance[4]    = 0.01;
  imu_msg.angular_velocity_covariance[8]    = 0.01;
  imu_msg.linear_acceleration_covariance[0] = 0.1;
  imu_msg.linear_acceleration_covariance[4] = 0.1;
  imu_msg.linear_acceleration_covariance[8] = 0.1;

  // Executor (1 handle: cmd_vel)
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_msg, &cmd_vel_callback, ON_NEW_DATA);

  prev_odom_time = millis();
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
  unsigned long now = millis();

  // Safety timeout: stop motors if no /cmd_vel for 1 second
  if (now - last_cmd_time > 1000) {
    setMotor(MOTOR_LEFT_ENA,  MOTOR_LEFT_INA,  MOTOR_LEFT_INB,  0);
    setMotor(MOTOR_RIGHT_ENA, MOTOR_RIGHT_INA, MOTOR_RIGHT_INB, 0);
  }

  // Odometry + IMU at 20 Hz (every 50 ms)
  if (now - prev_odom_time >= 50) {
    float dt = (now - prev_odom_time) / 1000.0f;
    prev_odom_time = now;

    // Atomic encoder snapshot — ESP32 equivalent of noInterrupts/interrupts
    portDISABLE_INTERRUPTS();
    long lticks = left_ticks;  left_ticks  = 0;
    long rticks = right_ticks; right_ticks = 0;
    portENABLE_INTERRUPTS();

    float d_left   = (lticks / TICKS_PER_REV) * (2.0f * PI * WHEEL_RADIUS);
    float d_right  = (rticks / TICKS_PER_REV) * (2.0f * PI * WHEEL_RADIUS);
    float d_center = (d_left + d_right) / 2.0f;
    float d_theta  = (d_right - d_left) / WHEEL_BASE;

    theta += d_theta;
    x_pos += d_center * cosf(theta);
    y_pos += d_center * sinf(theta);

    float v_linear  = d_center / dt;
    float v_angular = d_theta  / dt;

    int64_t time_ns = rmw_uros_epoch_nanos();

    // /wheel/odom
    odom_msg.header.stamp.sec       = time_ns / 1000000000LL;
    odom_msg.header.stamp.nanosec   = time_ns % 1000000000LL;
    odom_msg.header.frame_id.data   = frame_odom;
    odom_msg.header.frame_id.size   = strlen(frame_odom);
    odom_msg.child_frame_id.data    = frame_base_link;
    odom_msg.child_frame_id.size    = strlen(frame_base_link);
    odom_msg.pose.pose.position.x   = x_pos;
    odom_msg.pose.pose.position.y   = y_pos;
    odom_msg.pose.pose.position.z   = 0.0f;
    odom_msg.pose.pose.orientation.x = 0.0f;
    odom_msg.pose.pose.orientation.y = 0.0f;
    odom_msg.pose.pose.orientation.z = sinf(theta / 2.0f);
    odom_msg.pose.pose.orientation.w = cosf(theta / 2.0f);
    for (int i = 0; i < 36; i++) { odom_msg.pose.covariance[i] = 0.0; odom_msg.twist.covariance[i] = 0.0; }
    odom_msg.pose.covariance[0]   = 0.01;
    odom_msg.pose.covariance[7]   = 0.01;
    odom_msg.pose.covariance[35]  = 0.10;
    odom_msg.twist.twist.linear.x  = v_linear;
    odom_msg.twist.twist.angular.z = v_angular;
    odom_msg.twist.covariance[0]  = 0.02;
    odom_msg.twist.covariance[35] = 0.20;
    rcl_publish(&odom_pub, &odom_msg, NULL);

    // /imu/data (BNO055 via I2C on GPIO21/22)
    if (imu_ready) {
      imu::Quaternion quat  = bno.getQuat();
      imu::Vector<3>  gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      imu::Vector<3>  accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

      imu_msg.header.stamp.sec      = time_ns / 1000000000LL;
      imu_msg.header.stamp.nanosec  = time_ns % 1000000000LL;
      imu_msg.orientation.x         = quat.x();
      imu_msg.orientation.y         = quat.y();
      imu_msg.orientation.z         = quat.z();
      imu_msg.orientation.w         = quat.w();
      imu_msg.angular_velocity.x    = gyro.x();
      imu_msg.angular_velocity.y    = gyro.y();
      imu_msg.angular_velocity.z    = gyro.z();
      imu_msg.linear_acceleration.x = accel.x();
      imu_msg.linear_acceleration.y = accel.y();
      imu_msg.linear_acceleration.z = accel.z();
      rcl_publish(&imu_pub, &imu_msg, NULL);
    }
  }

  // Ultrasonic range at 10 Hz (every 100 ms)
  static unsigned long last_range_time = 0;
  if (now - last_range_time >= 100) {
    last_range_time = now;
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    unsigned long duration_us = pulseIn(ULTRASONIC_ECHO, HIGH, 25000UL);
    float distance_m = (duration_us == 0)
      ? range_msg.max_range
      : clamp(duration_us * 0.000172f, range_msg.min_range, range_msg.max_range);

    int64_t time_ns = rmw_uros_epoch_nanos();
    range_msg.header.stamp.sec     = time_ns / 1000000000LL;
    range_msg.header.stamp.nanosec = time_ns % 1000000000LL;
    range_msg.range = distance_m;
    rcl_publish(&range_pub, &range_msg, NULL);
  }

  // Spin ROS executor (processes incoming /cmd_vel)
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
