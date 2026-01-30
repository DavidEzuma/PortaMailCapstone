#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class StaticFramePublisher : public rclcpp::Node
{
public:
  explicit StaticFramePublisher()
  : Node("static_tf_broadcaster")
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // publish static transforms
    this->make_transforms();
  }

private:
  void make_transforms()
  {
    geometry_msgs::msg::TransformStamped t;

    // --- Transform: base_link -> laser ---
    // Adjust these values based on your physical mounting of the RPLIDAR A2M12
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "laser";

    // Translation (Meters)
    t.transform.translation.x = 0.1;  // 10cm forward
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.1;  // 10cm up

    // Rotation (Quaternion)
    tf2::Quaternion q;
    q.setRPY(0, 0, 0); // No rotation (flat)
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
    
    RCLCPP_INFO(this->get_logger(), "Published static transform: base_link -> laser");
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>());
  rclcpp::shutdown();
  return 0;
}