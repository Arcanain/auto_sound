#pragma once

#include <array>
#include <cstddef>
#include <future>
#include <limits>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include "auto_sound/auto_sound_component.hpp"

namespace auto_sound {

class AutoSoundNode : public rclcpp::Node {
 public:
  explicit AutoSoundNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void OnTimer();
  void PublishPathMarker();
  void PublishObstacleMarker();
  void PublishDetectionMarker();
  void OnObstacleFlag(const std_msgs::msg::Bool::SharedPtr msg);
  void StartSoundAsync(const std::string& sound_path, const char* label);
  bool IsSoundFinished();

  AutoSoundComponent component_{};
  rclcpp::Time last_time_{};
  rclcpp::TimerBase::SharedPtr timer_{};

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_{};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_{};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_{};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_pub_{};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detection_marker_pub_{};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_{};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_{};

  std::string frame_id_;
  std::string child_frame_id_;
  double publish_rate_hz_{30.0};
  double car_length_{4.0};
  double car_width_{1.8};
  double car_height_{1.4};
  std::array<float, 4> car_color_{{0.1f, 0.6f, 0.9f, 1.0f}};
  std::array<float, 4> path_color_{{1.0f, 0.7f, 0.1f, 1.0f}};
  double path_width_{0.15};
  double obstacle_x_{-7.5};
  double obstacle_y_{10.0};
  double obstacle_radius_{0.6};
  double obstacle_height_{1.2};
  std::array<float, 4> obstacle_color_{{0.85f, 0.2f, 0.2f, 1.0f}};
  double detection_radius_{1.0};
  double detection_line_width_{0.05};
  std::array<float, 4> detection_color_{{1.0f, 0.95f, 0.3f, 0.35f}};

  enum class RunState { kStartSound, kRunning, kGoalSound, kObstacleSound, kStopped };
  RunState state_{RunState::kRunning};
  std::string start_sound_path_{};
  std::string goal_sound_path_{};
  std::string left_sound_path_{};
  std::string right_sound_path_{};
  std::string obstacle_sound_path_{};
  std::string audio_player_{};
  bool enable_sound_{true};
  bool enable_turn_sound_{true};
  bool enable_obstacle_sound_{true};
  double turn_announce_distance_{1.5};
  double turn_min_angle_rad_{0.35};
  std::string obstacle_topic_{};
  std::future<int> sound_future_{};
  std::string sound_label_{};
  bool start_sound_started_{false};
  bool goal_sound_started_{false};
  bool obstacle_sound_started_{false};
  bool obstacle_pending_{false};
  bool obstacle_flag_{false};
  bool obstacle_auto_detect_{true};
  std::size_t last_turn_index_{std::numeric_limits<std::size_t>::max()};
};

}  // namespace auto_sound
