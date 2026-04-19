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

#include <yaml-cpp/yaml.h>

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------
enum class NavigationState { IDLE = 0, NAVIGATING = 1, COMPLETED = 3, FAILED = 4 };
enum class SystemMode      { MAPPING = 0, NAVIGATION = 1 };

struct Location { double x; double y; double w; };

// ---------------------------------------------------------------------------
class NavigationCoordinator : public rclcpp::Node {
public:
    using NavigateToPose  = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav   = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    NavigationCoordinator() : Node("navigation_coordinator") {
        this->declare_parameter("start_mode",      "navigation");
        this->declare_parameter("locations_file",  "");
        const char* home = std::getenv("HOME");
        std::string default_map_path = home
            ? std::string(home) + "/PortaMailCapstone/maps"
            : "/tmp/maps";
        this->declare_parameter("map_save_path", default_map_path);

        std::string mode_str = this->get_parameter("start_mode").as_string();
        if (mode_str == "mapping") {
            current_system_mode_ = SystemMode::MAPPING;
            RCLCPP_INFO(get_logger(), "System Start: MAPPING MODE (SLAM active)");
        } else {
            current_system_mode_ = SystemMode::NAVIGATION;
            RCLCPP_INFO(get_logger(), "System Start: NAVIGATION MODE (AMCL active)");
        }

        load_locations();

        subscription_ = create_subscription<std_msgs::msg::String>(
            "user_delivery_request", 10,
            std::bind(&NavigationCoordinator::user_input_callback, this, std::placeholders::_1));

        ui_feedback_pub_ = create_publisher<std_msgs::msg::String>("system_status", 10);

        nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        save_map_client_ = create_client<slam_toolbox::srv::SaveMap>("/slam_toolbox/save_map");

        current_nav_state_ = NavigationState::IDLE;
        RCLCPP_INFO(get_logger(), "Coordinator ready. Waiting for input...");
    }

private:
    SystemMode      current_system_mode_;
    NavigationState current_nav_state_;
    std::map<std::string, Location> locations_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    ui_feedback_pub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr       nav_action_client_;
    rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr  save_map_client_;

    // Stored so we can cancel an in-flight goal when a new request arrives.
    GoalHandleNav::SharedPtr current_goal_handle_;

    // -----------------------------------------------------------------------
    // Status publishing — always JSON so lcd_bridge can parse reliably.
    // {"type":"status","code":"ARRIVED","detail":""}
    // {"type":"error","code":"NAV_FAILED","detail":"..."}
    // -----------------------------------------------------------------------
    void publish_status(const std::string& code, const std::string& detail = "") {
        auto msg = std_msgs::msg::String();
        msg.data = "{\"type\":\"status\",\"code\":\"" + code
                 + "\",\"detail\":\"" + detail + "\"}";
        ui_feedback_pub_->publish(msg);
    }

    void publish_error(const std::string& code, const std::string& detail = "") {
        auto msg = std_msgs::msg::String();
        msg.data = "{\"type\":\"error\",\"code\":\"" + code
                 + "\",\"detail\":\"" + detail + "\"}";
        ui_feedback_pub_->publish(msg);
    }

    // -----------------------------------------------------------------------
    // Load locations.yaml — called at startup and before each navigation
    // so that waypoints saved during mapping are always current.
    // -----------------------------------------------------------------------
    void load_locations() {
        std::string file_path = get_parameter("locations_file").as_string();
        if (file_path.empty()) {
            RCLCPP_WARN(get_logger(), "locations_file not set — no locations loaded.");
            return;
        }
        try {
            YAML::Node config = YAML::LoadFile(file_path);
            if (config["locations"]) {
                locations_.clear();
                for (auto it = config["locations"].begin();
                     it != config["locations"].end(); ++it) {
                    std::string name = it->first.as<std::string>();
                    Location loc;
                    loc.x = it->second["x"].as<double>();
                    loc.y = it->second["y"].as<double>();
                    loc.w = it->second["w"].as<double>();
                    locations_[name] = loc;
                }
                RCLCPP_INFO(get_logger(), "Loaded %zu locations.", locations_.size());
            } else {
                RCLCPP_WARN(get_logger(),
                    "locations.yaml has no 'locations' key — run mapping mode to save waypoints.");
            }
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Failed to load locations YAML: %s", e.what());
        }
    }

    // -----------------------------------------------------------------------
    // Command dispatcher
    // -----------------------------------------------------------------------
    void user_input_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string command = msg->data;
        std::transform(command.begin(), command.end(), command.begin(), ::tolower);
        RCLCPP_INFO(get_logger(), "Received command: %s", command.c_str());

        if (current_system_mode_ == SystemMode::MAPPING) {
            if (command == "save_map") {
                trigger_save_map();
            } else {
                RCLCPP_WARN(get_logger(),
                    "Ignored '%s'. In Mapping Mode only 'save_map' is valid.", command.c_str());
            }
            return;
        }

        // Navigation mode
        if (command == "save_map") {
            RCLCPP_WARN(get_logger(), "Cannot save map in Navigation Mode.");
            return;
        }

        // Reload locations so any waypoints saved during the last mapping
        // session are visible without restarting the coordinator.
        load_locations();

        auto it = locations_.find(command);
        if (it != locations_.end()) {
            // Cancel any active goal before accepting the new destination.
            if (current_nav_state_ == NavigationState::NAVIGATING && current_goal_handle_) {
                RCLCPP_WARN(get_logger(),
                    "New destination received while navigating — canceling current goal.");
                current_goal_handle_->async_cancel_goal();
                current_goal_handle_.reset();
            }
            start_navigation(it->second);
        } else {
            RCLCPP_ERROR(get_logger(), "Unknown location: '%s'", command.c_str());
            publish_error("UNKNOWN_LOCATION", command);
        }
    }

    // -----------------------------------------------------------------------
    // Map save (mapping mode)
    // -----------------------------------------------------------------------
    void trigger_save_map() {
        publish_status("SAVING_MAP");

        if (!save_map_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "SLAM save_map service unavailable.");
            publish_error("SLAM_SERVICE_DOWN");
            return;
        }

        std::string base_path = get_parameter("map_save_path").as_string();
        auto request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
        // SLAM Toolbox appends .pgm and .yaml automatically.
        request->name.data = base_path + "/portamail_map";

        save_map_client_->async_send_request(request,
            [this](rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedFuture) {
                RCLCPP_INFO(get_logger(), "Map save request completed.");
                publish_status("MAP_SAVED");
            });
    }

    // -----------------------------------------------------------------------
    // Navigation (navigation mode)
    // -----------------------------------------------------------------------
    void start_navigation(const Location& loc) {
        current_nav_state_ = NavigationState::NAVIGATING;
        publish_status("NAVIGATING");

        if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "Nav2 action server not available!");
            publish_error("NAV2_OFFLINE");
            current_nav_state_ = NavigationState::FAILED;
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp    = now();
        goal_msg.pose.pose.position.x    = loc.x;
        goal_msg.pose.pose.position.y    = loc.y;
        goal_msg.pose.pose.orientation.w = loc.w;

        auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        opts.goal_response_callback =
            [this](const GoalHandleNav::SharedPtr& goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(get_logger(), "Goal rejected by Nav2.");
                    publish_error("GOAL_REJECTED");
                    current_nav_state_ = NavigationState::FAILED;
                    current_goal_handle_.reset();
                } else {
                    RCLCPP_INFO(get_logger(), "Goal accepted by Nav2.");
                    current_goal_handle_ = goal_handle;
                }
            };

        opts.feedback_callback =
            [this](GoalHandleNav::SharedPtr /*goal_handle*/,
                   const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
                if (!feedback) return;
                float dist = feedback->distance_remaining;
                RCLCPP_DEBUG(get_logger(), "Distance remaining: %.2f m", dist);
                // Publish coarse progress buckets so the LCD can show a
                // rough indicator without flooding the status topic.
                static float last_reported = -1.0f;
                if (last_reported < 0.0f || std::abs(dist - last_reported) >= 0.5f) {
                    last_reported = dist;
                    publish_status("NAVIGATING",
                        std::to_string(static_cast<int>(dist)) + "m");
                }
            };

        opts.result_callback =
            [this](const GoalHandleNav::WrappedResult& result) {
                current_goal_handle_.reset();
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(), "Navigation succeeded.");
                    publish_status("ARRIVED");
                    current_nav_state_ = NavigationState::COMPLETED;
                } else {
                    std::string detail;
                    switch (result.code) {
                        case rclcpp_action::ResultCode::ABORTED:   detail = "ABORTED";   break;
                        case rclcpp_action::ResultCode::CANCELED:  detail = "CANCELED";  break;
                        default:                                    detail = "UNKNOWN";   break;
                    }
                    RCLCPP_ERROR(get_logger(), "Navigation failed: %s", detail.c_str());
                    publish_error("NAV_FAILED", detail);
                    current_nav_state_ = NavigationState::FAILED;
                }
            };

        nav_action_client_->async_send_goal(goal_msg, opts);
    }
};

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationCoordinator>());
    rclcpp::shutdown();
    return 0;
}
