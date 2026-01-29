#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MockDriver : public rclcpp::Node
{
public:
  MockDriver()
  : Node("mock_teensy_driver")
  {
    // --- Robot Constants ---
    // Max Speed: 2 MPH ~= 0.89 m/s
    max_speed_mps_ = 0.89;

    // --- State ---
    x_ = 0.0;
    y_ = 0.0;
    th_ = 0.0;
    vx_ = 0.0;
    vth_ = 0.0;
    last_time_ = this->now();

    // --- Communication ---
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MockDriver::cmd_vel_callback, this, _1));

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Run physics loop at 20Hz (50ms)
    timer_ = this->create_wall_timer(
      50ms, std::bind(&MockDriver::update_physics, this));

    RCLCPP_INFO(this->get_logger(), "Mock C++ Driver Started (Simulating Physics)");
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // --- DEBUGGING LOG ---
    // This prints to the terminal every time a joystick command arrives
    RCLCPP_INFO(this->get_logger(), "Received Cmd: Linear=%.2f, Angular=%.2f", 
        msg->linear.x, msg->angular.z);

    // Update target velocities
    vx_ = msg->linear.x;
    vth_ = msg->angular.z;

    // Clamp speed (Simulate Firmware Safety)
    if (vx_ > max_speed_mps_) vx_ = max_speed_mps_;
    if (vx_ < -max_speed_mps_) vx_ = -max_speed_mps_;
  }

  void update_physics()
  {
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Simple Kinematic Integration
    double delta_x = (vx_ * cos(th_)) * dt;
    double delta_y = (vx_ * sin(th_)) * dt;
    double delta_th = vth_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    // Create Odometry Message
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    // Position
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    // Orientation (Quaternion from Yaw)
    tf2::Quaternion q;
    q.setRPY(0, 0, th_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // Velocity
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.angular.z = vth_;

    publisher_->publish(odom);
  }

  // Member Variables
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  double max_speed_mps_;
  double x_, y_, th_;
  double vx_, vth_;
  rclcpp::Time last_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockDriver>());
  rclcpp::shutdown();
  return 0;
}