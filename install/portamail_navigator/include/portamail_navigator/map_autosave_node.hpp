#ifndef PORTAMAIL_NAVIGATOR__MAP_AUTOSAVE_NODE_HPP_
#define PORTAMAIL_NAVIGATOR__MAP_AUTOSAVE_NODE_HPP_

#include <atomic>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "slam_toolbox/srv/save_map.hpp"

namespace portamail_navigator
{

class MapAutosaveNode : public rclcpp::Node
{
public:
  MapAutosaveNode();

private:
  void setup_timer_if_enabled();
  void trigger_save(const std::string & reason);
  std::string build_map_basename() const;
  bool ensure_output_directory() const;
  std::string default_output_directory() const;

  std::string save_service_;
  std::string output_directory_;
  std::string filename_prefix_;
  int autosave_interval_sec_;
  bool save_on_startup_;

  std::atomic<bool> save_in_progress_;
  rclcpp::TimerBase::SharedPtr autosave_timer_;
  rclcpp::Client<slam_toolbox::srv::SaveMap>::SharedPtr save_map_client_;
};

}  // namespace portamail_navigator

#endif  // PORTAMAIL_NAVIGATOR__MAP_AUTOSAVE_NODE_HPP_
