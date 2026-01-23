#include <memory>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <filesystem>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "slam_toolbox/srv/save_map.hpp"
#include "yaml-cpp/yaml.h"

// --- Constants & Enums ---
enum class NavigationState {
    IDLE = 0,
    NAVIGATING = 1,
    COMPLETED = 3,
    FAILED = 4
};

enum class SystemMode {
    MAPPING = 0,
    NAVIGATION = 1
};

struct Location {
    double x;
    double y;
    double w; // Treating this as Yaw (Theta) in radians for 2D navigation
};

class NavigationCoordinator : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigationCoordinator() : Node("navigation_coordinator") {
        // --- Parameters ---
        // Default path for WSL (Ubuntu 24.04 mapping to Windows)
        std::string default_path = std::filesystem::current_path().string();
        std::string default_yaml = default_path + "/src/locations.yaml";

        this->declare_parameter("start_mode", "navigation");
        this->declare_parameter("locations_file", default_yaml);
        this->declare_parameter("map_save_path", default_path);

        // --- System Mode Setup ---
        std::string mode_str = this->get_parameter("start_mode").as_string();
        if (mode_str == "mapping") {
            current_system_mode_ = SystemMode::MAPPING;
            RCLCPP_INFO(this->get_logger(), "System Start: MAPPING MODE (SLAM)");
        } else {
            current_system_mode_ = SystemMode::NAVIGATION;
            RCLCPP_INFO(this->get_logger(), "System Start: NAVIGATION MODE (AMCL)");
        }

        // --- Load YAML ---
        load_locations();

        // --- Communication Setup ---
        // Subscriber for User Input
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "user_delivery_request", 10,
            std::bind(&NavigationCoordinator::user_input_callback, this, std::placeholders::_1));

        // Publisher for Status Updates
        ui_feedback_pub_ = this->create_publisher<std_msgs::msg::String>("system_status", 10);

        // Action Client for Nav2
        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose");

        // Service Client for SLAM Toolbox
        save_map_client_ = this->create_client<slam_toolbox::srv::SaveMap>("/slam_toolbox/save_map");

        current_nav_state_ = NavigationState::IDLE;
        RCLCPP_INFO(this->get_logger(), "Coordinator Ready. Waiting for input...");
    }

private:
    // --- Member Variables ---
    SystemMode current_system_mode_;
    NavigationState current_nav_state_;
    std::map<std::string, Location> locations_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ui_feedback_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr save_map_client_;

    // --- Helper Methods ---

    void publish_status(const std::string &msg_txt) {
        auto msg = std_msgs::msg::String();
        msg.data = msg_txt;
        ui_feedback_pub_->publish(msg);
    }

    void load_locations() {
        std::string file_path = this->get_parameter("locations_file").as_string();
        try {
            YAML::Node config = YAML::LoadFile(file_path);
            
            if (config["locations"]) {
                for (YAML::const_iterator it = config["locations"].begin(); it != config["locations"].end(); ++it) {
                    std::string name = it->first.as<std::string>();
                    Location loc;
                    loc.x = it->second["x"].as<double>();
                    loc.y = it->second["y"].as<double>();
                    loc.w = it->second["w"].as<double>();
                    locations_[name] = loc;
                }
                RCLCPP_INFO(this->get_logger(), "Loaded %lu locations from YAML.", locations_.size());
            } else {
                RCLCPP_WARN(this->get_logger(), "YAML file missing 'locations' node.");
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load locations from %s: %s", file_path.c_str(), e.what());
        }
    }

    // --- Callbacks ---

    void user_input_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string command = msg->data;
        // Simple lowercase conversion
        std::transform(command.begin(), command.end(), command.begin(), ::tolower);
        RCLCPP_INFO(this->get_logger(), "Received command: %s", command.c_str());

        // 1. Handle Map Saving (Only in Mapping Mode)
        if (command == "save_map") {
            if (current_system_mode_ == SystemMode::MAPPING) {
                trigger_save_map();
            } else {
                RCLCPP_WARN(this->get_logger(), "Cannot save map in Navigation Mode.");
                publish_status("Error: Switch to Mapping Mode");
            }
            return;
        }

        // 2. Handle Navigation (Only in Navigation Mode)
        if (current_system_mode_ == SystemMode::MAPPING) {
            RCLCPP_WARN(this->get_logger(), "Ignored nav command (MAPPING active).");
            publish_status("Mode: Mapping. Nav Disabled.");
            return;
        }

        if (current_nav_state_ == NavigationState::NAVIGATING) {
            RCLCPP_WARN(this->get_logger(), "Busy navigating.");
            publish_status("Busy: Moving");
            return;
        }

        if (locations_.find(command) != locations_.end()) {
            Location target = locations_[command];
            start_navigation(target);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown location: %s", command.c_str());
            publish_status("Error: Unknown Location");
        }
    }

    // --- Logic: Map Saving ---

    void trigger_save_map() {
        publish_status("Status: Saving Map...");
        
        if (!save_map_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "SLAM service unavailable.");
            publish_status("Error: SLAM Service Down");
            return;
        }

        std::string base_path = this->get_parameter("map_save_path").as_string();
        // Use std::filesystem to ensure dir exists (C++17)
        // Ensure you compile with C++17
        
        std::string map_name = "office_map_v1";
        // Convert generic string to SaveMap/String type if needed.
        // The standard Slam Toolbox SaveMap.srv takes `std_msgs/String name`.
        
        auto request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
        request->name.data = base_path + "/" + map_name;

        auto future = save_map_client_->async_send_request(request, 
            [this](rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedFuture future) {
                // Lambda callback for service response
                try {
                    auto response = future.get();
                    // Assuming response indicates success (implementation dependent, usually non-zero or specific field)
                    RCLCPP_INFO(this->get_logger(), "Map save request completed.");
                    this->publish_status("Status: Map Saved");
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                    this->publish_status("Error: Save Failed");
                }
            });
    }

    // --- Logic: Navigation ---

    void start_navigation(const Location& loc) {
        current_nav_state_ = NavigationState::NAVIGATING;
        publish_status("Status: Computing Path...");

        if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 Action Server not available!");
            publish_status("Error: Nav2 Offline");
            current_nav_state_ = NavigationState::FAILED;
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = loc.x;
        goal_msg.pose.pose.position.y = loc.y;
        
        // Convert Yaw (loc.w) to Quaternion (z, w) for 2D planar rotation
        goal_msg.pose.pose.orientation.z = std::sin(loc.w / 2.0);
        goal_msg.pose.pose.orientation.w = std::cos(loc.w / 2.0);

        RCLCPP_INFO(this->get_logger(), "Sending goal: x=%f, y=%f", loc.x, loc.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        // Goal Response Callback
        send_goal_options.goal_response_callback =
            [this](const GoalHandleNav::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                    publish_status("Error: Goal Rejected");
                    current_nav_state_ = NavigationState::IDLE;
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
                    publish_status("Status: Moving...");
                }
            };

        // Feedback Callback
        send_goal_options.feedback_callback =
            [this](GoalHandleNav::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
                // Optional: Log distance remaining
                (void)feedback;
            };

        // Result Callback
        send_goal_options.result_callback =
            [this](const GoalHandleNav::WrappedResult & result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Goal reached!");
                        publish_status("Status: Arrived");
                        current_nav_state_ = NavigationState::COMPLETED;
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                        publish_status("Error: Path Aborted");
                        current_nav_state_ = NavigationState::FAILED;
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                        publish_status("Error: Nav Failed");
                        current_nav_state_ = NavigationState::FAILED;
                        break;
                }
                // Reset state to IDLE after completion
                current_nav_state_ = NavigationState::IDLE;
            };

        nav_action_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigationCoordinator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}