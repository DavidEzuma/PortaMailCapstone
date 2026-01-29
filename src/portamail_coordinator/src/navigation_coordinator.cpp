#include <memory>
#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <filesystem>
#include <algorithm> 

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "slam_toolbox/srv/save_map.hpp"

// System library
#include <yaml-cpp/yaml.h>

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
    double w;
};

class NavigationCoordinator : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigationCoordinator() : Node("navigation_coordinator") {
        // --- Parameters ---
        this->declare_parameter("start_mode", "navigation");
        this->declare_parameter("locations_file", "");
        this->declare_parameter("map_save_path", "/home/ubuntu/maps");

        // --- System Mode Setup ---
        std::string mode_str = this->get_parameter("start_mode").as_string();
        if (mode_str == "mapping") {
            current_system_mode_ = SystemMode::MAPPING;
            RCLCPP_INFO(this->get_logger(), "System Start: MAPPING MODE (SLAM active)");
        } else {
            current_system_mode_ = SystemMode::NAVIGATION;
            RCLCPP_INFO(this->get_logger(), "System Start: NAVIGATION MODE (AMCL active)");
        }

        // --- Load YAML ---
        load_locations();

        // --- Communication Setup ---
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "user_delivery_request", 10,
            std::bind(&NavigationCoordinator::user_input_callback, this, std::placeholders::_1));

        ui_feedback_pub_ = this->create_publisher<std_msgs::msg::String>("system_status", 10);

        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose");

        save_map_client_ = this->create_client<slam_toolbox::srv::SaveMap>("/slam_toolbox/save_map");

        current_nav_state_ = NavigationState::IDLE;
        RCLCPP_INFO(this->get_logger(), "Coordinator Ready. Waiting for user input...");
    }

private:
    SystemMode current_system_mode_;
    NavigationState current_nav_state_;
    std::map<std::string, Location> locations_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ui_feedback_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
    rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr save_map_client_;

    void publish_status(const std::string &msg_txt) {
        auto msg = std_msgs::msg::String();
        msg.data = msg_txt;
        ui_feedback_pub_->publish(msg);
    }

    void load_locations() {
        std::string file_path = this->get_parameter("locations_file").as_string();
        if (file_path.empty()) return;

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
                RCLCPP_INFO(this->get_logger(), "Loaded %lu locations.", locations_.size());
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML: %s", e.what());
        }
    }

    void user_input_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string command = msg->data;
        std::transform(command.begin(), command.end(), command.begin(), ::tolower);
        RCLCPP_INFO(this->get_logger(), "Received command: %s", command.c_str());

        // --- MAPPING MODE LOGIC ---
        if (current_system_mode_ == SystemMode::MAPPING) {
            if (command == "save_map") {
                trigger_save_map();
            } else {
                RCLCPP_WARN(this->get_logger(), "Ignored. In Mapping Mode, only 'save_map' is valid.");
            }
            return;
        }

        // --- NAVIGATION MODE LOGIC ---
        if (command == "save_map") {
            RCLCPP_WARN(this->get_logger(), "Cannot save map in Navigation Mode.");
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

    void trigger_save_map() {
        publish_status("Status: Saving Map...");
        
        if (!save_map_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "SLAM service unavailable.");
            publish_status("Error: SLAM Service Down");
            return;
        }

        std::string base_path = this->get_parameter("map_save_path").as_string();
        std::string map_name = "office_map_v1";
        
        auto request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
        // SLAM Toolbox adds .pgm and .yaml extensions automatically
        request->name.data = base_path + "/" + map_name;

        auto future = save_map_client_->async_send_request(request, 
            [this](rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedFuture future) {
                (void)future;
                RCLCPP_INFO(this->get_logger(), "Map save request sent.");
                this->publish_status("Status: Map Saved");
            });
    }

    void start_navigation(const Location& loc) {
        current_nav_state_ = NavigationState::NAVIGATING;
        publish_status("Status: Computing Path...");

        if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
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
        goal_msg.pose.pose.orientation.w = loc.w;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        
        send_goal_options.result_callback =
            [this](const GoalHandleNav::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    publish_status("Status: Arrived");
                    current_nav_state_ = NavigationState::COMPLETED;
                } else {
                    publish_status("Error: Nav Failed");
                    current_nav_state_ = NavigationState::FAILED;
                }
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