#pragma once

#include <array>
#include <cstddef>
#include <future>
#include <string>
#include <vector>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace auto_sound {

class AutoSoundNode : public rclcpp::Node {
 public:
  explicit AutoSoundNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void OnTimer();
  void OnOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void OnPath(const nav_msgs::msg::Path::SharedPtr msg);
  void PublishPathMarker();
  void PublishDetectionMarker();
  void StartSoundAsync(const std::string& sound_path, const std::string& label);
  bool IsSoundFinished();

  struct PathPoint {
    double x{0.0};
    double y{0.0};
  };

  struct OrderedSoundCue {
    double x{0.0};
    double y{0.0};
    std::string sound_path{};
    std::string label{};
  };

  rclcpp::TimerBase::SharedPtr timer_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_{};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_{};

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_{};
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr detection_marker_pub_{};

  std::string frame_id_;
  std::string path_topic_;
  std::string odom_topic_;
  double publish_rate_hz_{30.0};
  std::array<float, 4> path_color_{{1.0f, 0.7f, 0.1f, 1.0f}};
  double path_width_{0.15};
  double detection_radius_{1.0};
  double detection_line_width_{0.05};
  std::array<float, 4> detection_color_{{1.0f, 0.95f, 0.3f, 0.35f}};

  enum class RunState { kStartSound, kRunning };
  RunState state_{RunState::kRunning};
  std::string start_sound_path_{};
  std::string audio_player_{};
  bool enable_sound_{true};
  std::future<int> sound_future_{};
  std::string sound_label_{};
  bool start_sound_started_{false};
  double ordered_cue_radius_{1.0};
  std::vector<OrderedSoundCue> ordered_sound_cues_{};
  std::size_t next_ordered_cue_index_{0};
  std::vector<PathPoint> path_points_{};
  bool path_received_{false};
  double robot_x_{0.0};
  double robot_y_{0.0};
  bool odom_received_{false};
};

}  // namespace auto_sound
