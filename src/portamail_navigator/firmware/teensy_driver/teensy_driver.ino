#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

// --- HARDWARE PIN CONFIGURATION (Teensy 4.0) ---
// L298N Connections
#define MOTOR_LEFT_PWM  2
#define MOTOR_LEFT_IN1  3
#define MOTOR_LEFT_IN2  4

#define MOTOR_RIGHT_PWM 5
#define MOTOR_RIGHT_IN1 6
#define MOTOR_RIGHT_IN2 7

// FIT0186 Encoder Interrupt Pins
#define ENC_LEFT_A      8
#define ENC_LEFT_B      9
#define ENC_RIGHT_A     10
#define ENC_RIGHT_B     11

#define LED_PIN         13

// --- ROBOT CONSTANTS ---
// Adjust these after calibration!
const float WHEEL_RADIUS = 0.0325; // Meters (65mm wheels)
const float WHEEL_BASE = 0.20;     // Meters (Track width)
// FIT0186 (Metal Gearmotor) usually has ~341.2 ticks per output rev (check datasheet)
const float TICKS_PER_REV = 341.2; 

// --- SAFETY LIMITS ---
// 2 MPH ~= 0.894 m/s. 
const float MAX_SPEED_MPS = 0.89; 

// --- ROS 2 VARIABLES ---
rcl_subscription_t subscriber;
rcl_publisher_t odom_publisher;
geometry_msgs__msg__Twist msg;
nav_msgs__msg__Odometry odom_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// --- STATE VARIABLES ---
volatile long left_ticks = 0;
volatile long right_ticks = 0;
unsigned long prev_time = 0;
unsigned long last_cmd_time = 0;
float x_pos = 0.0;
float y_pos = 0.0;
float theta = 0.0;

// --- INTERRUPT HANDLERS ---
void doEncoderLeft() {
  if (digitalRead(ENC_LEFT_B) == HIGH) left_ticks++;
  else left_ticks--;
}

void doEncoderRight() {
  if (digitalRead(ENC_RIGHT_B) == HIGH) right_ticks++;
  else right_ticks--;
}

// --- HELPER: CLAMP VALUE ---
float clamp(float val, float min_val, float max_val) {
  if (val > max_val) return max_val;
  if (val < min_val) return min_val;
  return val;
}

// --- MOTOR CONTROL FUNCTIONS ---
void setMotor(int pwm_pin, int in1_pin, int in2_pin, float speed) {
  // Simple proportional control: 
  float pwm_factor = abs(speed) / MAX_SPEED_MPS; 
  int pwm_val = pwm_factor * 255;
  
  if (pwm_val > 255) pwm_val = 255;

  if (speed > 0.01) { // Forward with deadzone
    digitalWrite(in1_pin, HIGH);
    digitalWrite(in2_pin, LOW);
  } else if (speed < -0.01) { // Reverse with deadzone
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, HIGH);
  } else { // Stop
    digitalWrite(in1_pin, LOW);
    digitalWrite(in2_pin, LOW);
    pwm_val = 0;
  }
  analogWrite(pwm_pin, pwm_val);
}

// --- ROS CALLBACK: CMD_VEL ---
void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  // Visual Debug: Blink LED when command received
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  last_cmd_time = millis();

  // 1. Clamp Linear Speed
  float linear = clamp(msg->linear.x, -MAX_SPEED_MPS, MAX_SPEED_MPS);
  float angular = msg->angular.z;

  // 2. Differential Drive Kinematics
  float left_speed = linear - (angular * WHEEL_BASE / 2.0);
  float right_speed = linear + (angular * WHEEL_BASE / 2.0);

  // 3. Drive Motors
  setMotor(MOTOR_LEFT_PWM, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, left_speed);
  setMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, right_speed);
}

// --- SETUP ---
void setup() {
  // 1. Hardware Setup
  pinMode(LED_PIN, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), doEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), doEncoderRight, RISING);

  // 2. micro-ROS Setup
  set_microros_transports(); // Serial (USB)
  
  delay(2000); 

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "teensy_driver", "", &support);

  // Sync Time
  rmw_uros_sync_session(1000);

  // 3. Subscriber (Receive Commands)
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");

  // 4. Publisher (Send Odometry)
  // Note: Publishing to /wheel/odom for EKF consumption
  rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "wheel/odom");

  // 5. Executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

// --- LOOP ---
void loop() {
  // 1. Safety Timeout (Stop if no command for 1 second)
  if (millis() - last_cmd_time > 1000) {
    setMotor(MOTOR_LEFT_PWM, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, 0);
    setMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, 0);
  }

  // 2. Calculate Odometry
  unsigned long current_time = millis();
  if (current_time - prev_time >= 50) { // 20Hz update rate
    float dt = (current_time - prev_time) / 1000.0;
    
    long current_left_ticks = left_ticks;
    long current_right_ticks = right_ticks;
    left_ticks = 0; 
    right_ticks = 0;

    // Ticks -> Distance
    float d_left = (current_left_ticks / TICKS_PER_REV) * (2 * PI * WHEEL_RADIUS);
    float d_right = (current_right_ticks / TICKS_PER_REV) * (2 * PI * WHEEL_RADIUS);

    float d_center = (d_left + d_right) / 2.0;
    float d_theta = (d_right - d_left) / WHEEL_BASE;

    theta += d_theta;
    x_pos += d_center * cos(theta);
    y_pos += d_center * sin(theta);

    // Populate Odometry Message
    int64_t time_ns = rmw_uros_epoch_nanos();
    odom_msg.header.stamp.sec = time_ns / 1000000000;
    odom_msg.header.stamp.nanosec = time_ns % 1000000000;
    
    // Frame IDs
    odom_msg.header.frame_id.data = (char*)"odom";
    odom_msg.header.frame_id.size = strlen("odom");
    odom_msg.child_frame_id.data = (char*)"base_link";
    odom_msg.child_frame_id.size = strlen("base_link");

    // Position & Orientation
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.orientation.z = sin(theta / 2.0); 
    odom_msg.pose.pose.orientation.w = cos(theta / 2.0);
    
    // Publish
    rcl_publish(&odom_publisher, &odom_msg, NULL);
    
    prev_time = current_time;
  }

  // 3. Spin ROS
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
}