#include "portamail_navigator/map_autosave_node.hpp"

#include <chrono>
#include <ctime>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <memory>
#include <sstream>
#include <utility>

namespace portamail_navigator
{

MapAutosaveNode::MapAutosaveNode()
: rclcpp::Node("map_autosave_node"), save_in_progress_(false)
{
  save_service_ = declare_parameter<std::string>("save_service", "/slam_toolbox/save_map");
  output_directory_ = declare_parameter<std::string>("output_directory", default_output_directory());
  filename_prefix_ = declare_parameter<std::string>("filename_prefix", "portamail_map");
  autosave_interval_sec_ = declare_parameter<int>("autosave_interval_sec", 30);
  save_on_startup_ = declare_parameter<bool>("save_on_startup", false);

  save_map_client_ = create_client<slam_toolbox::srv::SaveMap>(save_service_);

  if (!ensure_output_directory()) {
    RCLCPP_ERROR(
      get_logger(),
      "Map auto-save disabled because output directory is invalid: %s",
      output_directory_.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Map auto-save output directory: %s", output_directory_.c_str());
  RCLCPP_INFO(get_logger(), "Map auto-save interval: %d seconds", autosave_interval_sec_);

  if (save_on_startup_) {
    trigger_save("startup");
  }

  setup_timer_if_enabled();
}

void MapAutosaveNode::setup_timer_if_enabled()
{
  if (autosave_interval_sec_ <= 0) {
    RCLCPP_WARN(
      get_logger(),
      "autosave_interval_sec <= 0. Automatic periodic saves are disabled.");
    return;
  }

  autosave_timer_ = create_wall_timer(
    std::chrono::seconds(autosave_interval_sec_),
    [this]() {
      trigger_save("periodic");
    });
}

void MapAutosaveNode::trigger_save(const std::string & reason)
{
  if (save_in_progress_.exchange(true)) {
    RCLCPP_WARN(get_logger(), "Map save skipped (%s): previous save still in progress.", reason.c_str());
    return;
  }

  if (!save_map_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(
      get_logger(),
      "Map save skipped (%s): save service %s not available yet.",
      reason.c_str(),
      save_service_.c_str());
    save_in_progress_.store(false);
    return;
  }

  auto request = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
  request->name.data = build_map_basename();

  RCLCPP_INFO(
    get_logger(),
    "Saving map (%s) to base path: %s",
    reason.c_str(),
    request->name.data.c_str());

  save_map_client_->async_send_request(
    request,
    [this, reason, base_path = request->name.data](
      rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedFuture response_future) {
      try {
        (void)response_future.get();
        RCLCPP_INFO(
          get_logger(),
          "Map save complete (%s): %s.{yaml,pgm}",
          reason.c_str(),
          base_path.c_str());
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(
          get_logger(),
          "Map save failed (%s): %s",
          reason.c_str(),
          exception.what());
      }
      save_in_progress_.store(false);
    });
}

std::string MapAutosaveNode::build_map_basename() const
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);

  std::tm local_tm{};
#if defined(_WIN32)
  localtime_s(&local_tm, &now_time);
#else
  localtime_r(&now_time, &local_tm);
#endif

  std::ostringstream timestamp;
  timestamp << std::put_time(&local_tm, "%Y%m%d_%H%M%S");

  const std::filesystem::path output_path(output_directory_);
  const std::filesystem::path base_name = output_path / (filename_prefix_ + "_" + timestamp.str());
  return base_name.string();
}

bool MapAutosaveNode::ensure_output_directory() const
{
  try {
    const std::filesystem::path output_path(output_directory_);
    if (std::filesystem::exists(output_path)) {
      return std::filesystem::is_directory(output_path);
    }
    return std::filesystem::create_directories(output_path);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to create map output directory %s: %s",
      output_directory_.c_str(),
      exception.what());
    return false;
  }
}

std::string MapAutosaveNode::default_output_directory() const
{
  const char * home_dir = std::getenv("HOME");
  if (home_dir != nullptr) {
    const std::filesystem::path home_path(home_dir);
    const std::filesystem::path portamail_ws = home_path / "PortaMailCapstone";
    if (std::filesystem::exists(portamail_ws)) {
      return (portamail_ws / "maps").string();
    }
    return (home_path / "portamail_maps").string();
  }
  return "/tmp/portamail_maps";
}

}  // namespace portamail_navigator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<portamail_navigator::MapAutosaveNode>());
  rclcpp::shutdown();
  return 0;
}
