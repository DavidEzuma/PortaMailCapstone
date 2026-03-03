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
// HARDWARE PIN CONFIGURATION (Teensy 4.0)
// =============================================================================

// STMicro motor driver (MultiPowerSO-30, 5.5-24V): INA=IN1, INB=IN2, PWM
#define MOTOR_LEFT_PWM   2
#define MOTOR_LEFT_IN1   3
#define MOTOR_LEFT_IN2   4
#define MOTOR_RIGHT_PWM  5
#define MOTOR_RIGHT_IN1  6
#define MOTOR_RIGHT_IN2  7

// FIT0186 quadrature encoder interrupt pins (DigiKey 1738-1106-ND)
// Rising edge on A, direction from B
#define ENC_LEFT_A   8
#define ENC_LEFT_B   9
#define ENC_RIGHT_A  10
#define ENC_RIGHT_B  11

#define LED_PIN  13

// BNO055 IMU (DigiKey 1528-1426-ND) via I2C
// Teensy 4.0 Wire default: SDA=18, SCL=19 — I2C address 0x28
// (No pin defines needed; Wire.begin() uses pins 18/19 by default)

// Ultrasonic ranger (GR-USNRG / HC-SR04-style)
// IMPORTANT: HC-SR04 Echo outputs 5V. Use a voltage divider or level shifter
//            before connecting Echo to Teensy 4.0 (3.3V-tolerant GPIO).
#define ULTRASONIC_TRIG  27  // Trigger: send 10µs HIGH pulse
#define ULTRASONIC_ECHO  26  // Echo: measure pulse duration for distance
                             // TODO: confirm ECHO pin against final PCB

// =============================================================================
// ROBOT CONSTANTS  (calibrate on physical robot before use)
// =============================================================================

// Wheel: 1/10 RC Monster Truck 2.8" (71.1mm diameter) -> radius = 35.56mm
const float WHEEL_RADIUS = 0.03556f; // meters

// Measured track width (centre-to-centre of drive wheels)
// *** MEASURE THE ACTUAL TRACK WIDTH AND UPDATE THIS VALUE ***
const float WHEEL_BASE = 0.20f; // meters

// FIT0186: 8 PPR on motor shaft, 90:1 gear ratio.
// Theoretical single-edge (rising on A only): 8 x 90 = 720 ticks/output_rev.
// *** CALIBRATE: drive exactly 1 m, log tick counts, then adjust. ***
const float TICKS_PER_REV = 720.0f;

const float MAX_SPEED_MPS = 0.89f; // 2 MPH safety limit
const int   PWM_MIN = 70;          // Minimum PWM to overcome motor dead zone
const int   PWM_MAX = 255;

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
unsigned long prev_odom_time  = 0;
unsigned long last_cmd_time   = 0;
float x_pos = 0.0f;
float y_pos = 0.0f;
float theta = 0.0f;

// Static frame ID strings (must persist for the lifetime of the message)
static char frame_odom[]        = "odom";
static char frame_base_link[]   = "base_link";
static char frame_imu_link[]    = "imu_link";
static char frame_ultrasonic[]  = "ultrasonic_link";

// =============================================================================
// ENCODER INTERRUPT HANDLERS
// =============================================================================
void doEncoderLeft() {
  if (digitalRead(ENC_LEFT_B) == HIGH) left_ticks++;
  else                                  left_ticks--;
}
void doEncoderRight() {
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

void setMotor(int pwm_pin, int in1, int in2, float speed) {
  int pwm_val = 0;
  if (fabsf(speed) > 0.01f) {
    float factor = fabsf(speed) / MAX_SPEED_MPS;
    pwm_val = PWM_MIN + (int)(factor * (PWM_MAX - PWM_MIN));
    if (pwm_val > PWM_MAX) pwm_val = PWM_MAX;
  }
  if      (speed >  0.01f) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else if (speed < -0.01f) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                     { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); pwm_val = 0; }
  analogWrite(pwm_pin, pwm_val);
}

// =============================================================================
// ROS CALLBACK: /cmd_vel
// =============================================================================
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // blink for debug
  last_cmd_time = millis();

  float linear  = clamp(msg->linear.x,  -MAX_SPEED_MPS, MAX_SPEED_MPS);
  float angular = msg->angular.z;

  float left_spd  = linear - (angular * WHEEL_BASE / 2.0f);
  float right_spd = linear + (angular * WHEEL_BASE / 2.0f);
  left_spd  = clamp(left_spd,  -MAX_SPEED_MPS, MAX_SPEED_MPS);
  right_spd = clamp(right_spd, -MAX_SPEED_MPS, MAX_SPEED_MPS);

  setMotor(MOTOR_LEFT_PWM,  MOTOR_LEFT_IN1,  MOTOR_LEFT_IN2,  left_spd);
  setMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, right_spd);
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  // --- GPIO ---
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_PWM,  OUTPUT); pinMode(MOTOR_LEFT_IN1,  OUTPUT); pinMode(MOTOR_LEFT_IN2,  OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT); pinMode(MOTOR_RIGHT_IN1, OUTPUT); pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  pinMode(ENC_LEFT_A,  INPUT_PULLUP); pinMode(ENC_LEFT_B,  INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP); pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A),  doEncoderLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), doEncoderRight, RISING);
  pinMode(ULTRASONIC_TRIG, OUTPUT); digitalWrite(ULTRASONIC_TRIG, LOW);
  pinMode(ULTRASONIC_ECHO, INPUT);

  // --- BNO055 (I2C: SDA=18, SCL=19) ---
  Wire.begin();
  imu_ready = bno.begin();
  if (imu_ready) {
    bno.setExtCrystalUse(true);
  }

  // --- micro-ROS (USB Serial: Pi USB-A -> Teensy Micro-USB -> /dev/ttyACM0) ---
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "teensy_driver", "", &support);
  rmw_uros_sync_session(1000);

  // --- Subscriber: /cmd_vel ---
  rclc_subscription_init_default(
    &cmd_vel_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

  // --- Publisher: /wheel/odom ---
  rclc_publisher_init_default(
    &odom_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "wheel/odom");

  // --- Publisher: /imu/data ---
  rclc_publisher_init_default(
    &imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");

  // --- Publisher: /ultrasonic/range ---
  rclc_publisher_init_default(
    &range_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "ultrasonic/range");

  // --- Static Range message fields (set once) ---
  range_msg.radiation_type  = sensor_msgs__msg__Range__ULTRASOUND;
  range_msg.field_of_view   = 0.26f; // ~15 degrees (typical HC-SR04)
  range_msg.min_range       = 0.02f; // 2 cm
  range_msg.max_range       = 4.00f; // 4 m
  range_msg.header.frame_id.data     = frame_ultrasonic;
  range_msg.header.frame_id.size     = strlen(frame_ultrasonic);
  range_msg.header.frame_id.capacity = strlen(frame_ultrasonic) + 1;

  // --- Static IMU message covariances (set once) ---
  // BNO055 orientation accuracy spec: ±5 deg = 0.087 rad -> variance ~0.0076
  imu_msg.header.frame_id.data     = frame_imu_link;
  imu_msg.header.frame_id.size     = strlen(frame_imu_link);
  imu_msg.header.frame_id.capacity = strlen(frame_imu_link) + 1;
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i]         = 0.0;
    imu_msg.angular_velocity_covariance[i]    = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }
  imu_msg.orientation_covariance[0]         = 0.0076; // roll
  imu_msg.orientation_covariance[4]         = 0.0076; // pitch
  imu_msg.orientation_covariance[8]         = 0.0076; // yaw
  imu_msg.angular_velocity_covariance[0]    = 0.01;
  imu_msg.angular_velocity_covariance[4]    = 0.01;
  imu_msg.angular_velocity_covariance[8]    = 0.01;
  imu_msg.linear_acceleration_covariance[0] = 0.1;
  imu_msg.linear_acceleration_covariance[4] = 0.1;
  imu_msg.linear_acceleration_covariance[8] = 0.1;

  // --- Executor (1 handle: cmd_vel subscriber) ---
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_msg, &cmd_vel_callback, ON_NEW_DATA);

  prev_odom_time = millis();
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
  unsigned long now = millis();

  // --- Safety timeout: stop motors if no cmd_vel for 1 second ---
  if (now - last_cmd_time > 1000) {
    setMotor(MOTOR_LEFT_PWM,  MOTOR_LEFT_IN1,  MOTOR_LEFT_IN2,  0);
    setMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, 0);
  }

  // --- Odometry + IMU at 20 Hz (every 50 ms) ---
  if (now - prev_odom_time >= 50) {
    float dt = (now - prev_odom_time) / 1000.0f;
    prev_odom_time = now;

    // Atomic encoder read
    noInterrupts();
    long lticks = left_ticks;  left_ticks  = 0;
    long rticks = right_ticks; right_ticks = 0;
    interrupts();

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

    // Publish /wheel/odom
    odom_msg.header.stamp.sec      = time_ns / 1000000000LL;
    odom_msg.header.stamp.nanosec  = time_ns % 1000000000LL;
    odom_msg.header.frame_id.data     = frame_odom;
    odom_msg.header.frame_id.size     = strlen(frame_odom);
    odom_msg.child_frame_id.data      = frame_base_link;
    odom_msg.child_frame_id.size      = strlen(frame_base_link);
    odom_msg.pose.pose.position.x     = x_pos;
    odom_msg.pose.pose.position.y     = y_pos;
    odom_msg.pose.pose.position.z     = 0.0f;
    odom_msg.pose.pose.orientation.x  = 0.0f;
    odom_msg.pose.pose.orientation.y  = 0.0f;
    odom_msg.pose.pose.orientation.z  = sinf(theta / 2.0f);
    odom_msg.pose.pose.orientation.w  = cosf(theta / 2.0f);
    for (int i = 0; i < 36; i++) odom_msg.pose.covariance[i]  = 0.0;
    for (int i = 0; i < 36; i++) odom_msg.twist.covariance[i] = 0.0;
    odom_msg.pose.covariance[0]   = 0.01;  // x variance (m^2)
    odom_msg.pose.covariance[7]   = 0.01;  // y variance (m^2)
    odom_msg.pose.covariance[35]  = 0.10;  // yaw variance (rad^2)
    odom_msg.twist.twist.linear.x  = v_linear;
    odom_msg.twist.twist.angular.z = v_angular;
    odom_msg.twist.covariance[0]  = 0.02;  // vx variance
    odom_msg.twist.covariance[35] = 0.20;  // vyaw variance
    rcl_publish(&odom_pub, &odom_msg, NULL);

    // Publish /imu/data (BNO055 via I2C on Teensy)
    if (imu_ready) {
      imu::Quaternion quat  = bno.getQuat();
      imu::Vector<3>  gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);     // rad/s
      imu::Vector<3>  accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);   // m/s^2 (gravity removed)

      imu_msg.header.stamp.sec     = time_ns / 1000000000LL;
      imu_msg.header.stamp.nanosec = time_ns % 1000000000LL;
      imu_msg.orientation.x        = quat.x();
      imu_msg.orientation.y        = quat.y();
      imu_msg.orientation.z        = quat.z();
      imu_msg.orientation.w        = quat.w();
      imu_msg.angular_velocity.x   = gyro.x();
      imu_msg.angular_velocity.y   = gyro.y();
      imu_msg.angular_velocity.z   = gyro.z();
      imu_msg.linear_acceleration.x = accel.x();
      imu_msg.linear_acceleration.y = accel.y();
      imu_msg.linear_acceleration.z = accel.z();
      rcl_publish(&imu_pub, &imu_msg, NULL);
    }
  }

  // --- Ultrasonic range at 10 Hz (every 100 ms) ---
  static unsigned long last_range_time = 0;
  if (now - last_range_time >= 100) {
    last_range_time = now;

    // Send 10 µs trigger pulse
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);

    // Measure echo: 25 ms timeout covers ~4.3 m max range
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

  // --- Spin ROS executor (processes incoming /cmd_vel) ---
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}
