#include "auto_sound/auto_sound_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <future>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace auto_sound {

namespace {

constexpr double kPi = 3.14159265358979323846;

std::vector<Waypoint> BuildPath(const std::vector<double>& flat) {
  if (flat.size() < 4 || (flat.size() % 2) != 0) {
    return {};
  }
  std::vector<Waypoint> path;
  path.reserve(flat.size() / 2);
  for (size_t i = 0; i + 1 < flat.size(); i += 2) {
    path.push_back(Waypoint{flat[i], flat[i + 1]});
  }
  if (path.size() < 2) {
    return {};
  }
  return path;
}

std::array<float, 4> ToColor(const std::vector<double>& values,
                             const std::array<float, 4>& fallback) {
  if (values.size() != 4) {
    return fallback;
  }
  std::array<float, 4> result{};
  for (size_t i = 0; i < 4; ++i) {
    result[i] = static_cast<float>(values[i]);
  }
  return result;
}

double NormalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

enum class TurnDirection { kLeft, kRight, kStraight };

TurnDirection ComputeTurnDirection(const std::vector<Waypoint>& path,
                                   std::size_t index,
                                   double min_angle_rad) {
  if (index == 0 || index + 1 >= path.size()) {
    return TurnDirection::kStraight;
  }
  const auto& prev = path[index - 1];
  const auto& curr = path[index];
  const auto& next = path[index + 1];

  const double v1x = curr.x - prev.x;
  const double v1y = curr.y - prev.y;
  const double v2x = next.x - curr.x;
  const double v2y = next.y - curr.y;

  const double len1 = std::hypot(v1x, v1y);
  const double len2 = std::hypot(v2x, v2y);
  if (len1 < 1e-6 || len2 < 1e-6) {
    return TurnDirection::kStraight;
  }

  const double angle1 = std::atan2(v1y, v1x);
  const double angle2 = std::atan2(v2y, v2x);
  const double delta = NormalizeAngle(angle2 - angle1);

  if (std::fabs(delta) < min_angle_rad) {
    return TurnDirection::kStraight;
  }

  const double cross = v1x * v2y - v1y * v2x;
  if (cross > 0.0) {
    return TurnDirection::kLeft;
  }
  if (cross < 0.0) {
    return TurnDirection::kRight;
  }
  return TurnDirection::kStraight;
}

bool FileExists(const std::string& path) {
  std::ifstream file(path);
  return file.good();
}

std::string ShellQuote(const std::string& input) {
  std::string out = "'";
  for (char c : input) {
    if (c == '\'') {
      out += "'\\''";
    } else {
      out += c;
    }
  }
  out += "'";
  return out;
}

std::string ResolveSoundPath(const std::string& input, const std::string& pkg_share) {
  if (input.empty()) {
    return {};
  }
  if (!input.empty() && input.front() == '/') {
    return input;
  }
  if (input.rfind("sounds/", 0) == 0) {
    return pkg_share + "/" + input;
  }
  return pkg_share + "/sounds/" + input;
}

std::string ExtractCommandName(const std::string& command) {
  const auto start = command.find_first_not_of(" \t");
  if (start == std::string::npos) {
    return {};
  }
  const auto end = command.find_first_of(" \t", start);
  return command.substr(start, end - start);
}

bool CommandExists(const std::string& command) {
  const std::string name = ExtractCommandName(command);
  if (name.empty()) {
    return false;
  }
  if (name.find('/') != std::string::npos) {
    return FileExists(name);
  }
  const std::string check = "command -v " + ShellQuote(name) + " >/dev/null 2>&1";
  return std::system(check.c_str()) == 0;
}

std::string DetectAudioPlayer() {
  struct Candidate {
    const char* binary;
    const char* command;
  };
  const Candidate candidates[] = {
      {"ffplay", "ffplay -nodisp -autoexit -loglevel error"},
      {"gst-play-1.0", "gst-play-1.0 --quiet --no-interactive"},
      {"mpg123", "mpg123 -q"},
      {"mpv", "mpv --no-video --really-quiet"},
      {"cvlc", "cvlc --intf dummy --play-and-exit --quiet"},
      {"vlc", "vlc --intf dummy --play-and-exit --quiet"},
      {"play", "play -q"},
  };

  for (const auto& candidate : candidates) {
    if (CommandExists(candidate.binary)) {
      return candidate.command;
    }
  }
  return {};
}

}  // namespace

AutoSoundNode::AutoSoundNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("auto_sound_node", options) {
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);

  const double speed_mps = declare_parameter<double>("speed_mps", 1.0);
  const double start_x = declare_parameter<double>("start_x", 0.0);
  const double start_y = declare_parameter<double>("start_y", 0.0);
  const double start_yaw = declare_parameter<double>("start_yaw", 0.0);
  const double waypoint_tolerance = declare_parameter<double>("waypoint_tolerance", 0.3);
  const double max_yaw_rate = declare_parameter<double>("max_yaw_rate", 1.0);

  car_length_ = declare_parameter<double>("car_length", 4.0);
  car_width_ = declare_parameter<double>("car_width", 1.8);
  car_height_ = declare_parameter<double>("car_height", 1.4);
  path_width_ = declare_parameter<double>("path_width", 0.15);
  obstacle_x_ = declare_parameter<double>("obstacle_x", -7.5);
  obstacle_y_ = declare_parameter<double>("obstacle_y", 10.0);
  obstacle_radius_ = declare_parameter<double>("obstacle_radius", 0.6);
  obstacle_height_ = declare_parameter<double>("obstacle_height", 1.2);
  detection_radius_ = declare_parameter<double>("detection_radius", 1.0);
  detection_line_width_ = declare_parameter<double>("detection_line_width", 0.05);

  const auto path_points = declare_parameter<std::vector<double>>(
      "path_points", std::vector<double>{});
  const bool auto_align_start_yaw = declare_parameter<bool>("auto_align_start_yaw", true);

  enable_sound_ = declare_parameter<bool>("enable_sound", true);
  audio_player_ = declare_parameter<std::string>("audio_player", "auto");
  const auto start_sound = declare_parameter<std::string>("start_sound", "start.wav");
  const auto goal_sound = declare_parameter<std::string>("goal_sound", "goal.wav");
  enable_turn_sound_ = declare_parameter<bool>("enable_turn_sound", true);
  turn_announce_distance_ = declare_parameter<double>("turn_announce_distance", 1.5);
  const double turn_min_angle_deg = declare_parameter<double>("turn_min_angle_deg", 20.0);
  const auto left_sound = declare_parameter<std::string>("left_sound", "left.wav");
  const auto right_sound = declare_parameter<std::string>("right_sound", "right.wav");
  enable_obstacle_sound_ = declare_parameter<bool>("enable_obstacle_sound", true);
  const auto obstacle_sound =
      declare_parameter<std::string>("obstacle_sound", "obstacle.wav");
  obstacle_topic_ = declare_parameter<std::string>("obstacle_topic", "obstacle_detected");
  obstacle_auto_detect_ = declare_parameter<bool>("obstacle_auto_detect", true);

  const auto car_color = declare_parameter<std::vector<double>>(
      "car_color_rgba", std::vector<double>{0.1, 0.6, 0.9, 1.0});
  const auto path_color = declare_parameter<std::vector<double>>(
      "path_color_rgba", std::vector<double>{1.0, 0.7, 0.1, 1.0});
  const auto obstacle_color = declare_parameter<std::vector<double>>(
      "obstacle_color_rgba", std::vector<double>{0.85, 0.2, 0.2, 1.0});
  const auto detection_color = declare_parameter<std::vector<double>>(
      "detection_color_rgba", std::vector<double>{1.0, 0.95, 0.3, 0.35});
  car_color_ = ToColor(car_color, car_color_);
  path_color_ = ToColor(path_color, path_color_);
  obstacle_color_ = ToColor(obstacle_color, obstacle_color_);
  detection_color_ = ToColor(detection_color, detection_color_);

  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "publish_rate_hz must be > 0.0. Using 30.0.");
    publish_rate_hz_ = 30.0;
  }

  const auto path = BuildPath(path_points);
  if (path.size() < 2) {
    RCLCPP_WARN(get_logger(),
                "path_points is empty or invalid. Provide at least two waypoints.");
  }
  double initial_yaw = start_yaw;
  if (auto_align_start_yaw && path.size() >= 2) {
    const auto& p0 = path[0];
    const auto& p1 = path[1];
    const double dx = p1.x - p0.x;
    const double dy = p1.y - p0.y;
    if (std::hypot(dx, dy) > 1e-6) {
      initial_yaw = std::atan2(dy, dx);
    }
  }

  AutoSoundComponent::Params params;
  params.speed_mps = speed_mps;
  params.start_x = start_x;
  params.start_y = start_y;
  params.start_yaw = initial_yaw;
  params.waypoint_tolerance = waypoint_tolerance;
  params.max_yaw_rate = max_yaw_rate;
  component_.Reset(params);
  component_.SetPath(path);
  last_turn_index_ = std::numeric_limits<std::size_t>::max();

  const std::string pkg_share =
      ament_index_cpp::get_package_share_directory("auto_sound");
  start_sound_path_ = ResolveSoundPath(start_sound, pkg_share);
  goal_sound_path_ = ResolveSoundPath(goal_sound, pkg_share);
  left_sound_path_ = ResolveSoundPath(left_sound, pkg_share);
  right_sound_path_ = ResolveSoundPath(right_sound, pkg_share);
  obstacle_sound_path_ = ResolveSoundPath(obstacle_sound, pkg_share);

  turn_min_angle_rad_ = turn_min_angle_deg * kPi / 180.0;
  if (turn_announce_distance_ < 0.0) {
    turn_announce_distance_ = 0.0;
  }

  if (enable_sound_) {
    if (audio_player_ == "auto") {
      audio_player_ = DetectAudioPlayer();
    }
    if (audio_player_.empty()) {
      RCLCPP_WARN(get_logger(),
                  "No audio player found. Install ffplay/gst-play-1.0/mpg123/mpv/cvlc or set "
                  "audio_player. Disabling sound.");
      enable_sound_ = false;
    } else if (!CommandExists(audio_player_)) {
      RCLCPP_WARN(get_logger(),
                  "Audio player command not found: %s. Disabling sound.",
                  audio_player_.c_str());
      enable_sound_ = false;
    }
  }

  if (!start_sound_path_.empty() && !FileExists(start_sound_path_)) {
    RCLCPP_WARN(get_logger(), "Start sound not found: %s", start_sound_path_.c_str());
    start_sound_path_.clear();
  }
  if (!goal_sound_path_.empty() && !FileExists(goal_sound_path_)) {
    RCLCPP_WARN(get_logger(), "Goal sound not found: %s", goal_sound_path_.c_str());
    goal_sound_path_.clear();
  }
  if (!left_sound_path_.empty() && !FileExists(left_sound_path_)) {
    RCLCPP_WARN(get_logger(), "Left turn sound not found: %s", left_sound_path_.c_str());
    left_sound_path_.clear();
  }
  if (!right_sound_path_.empty() && !FileExists(right_sound_path_)) {
    RCLCPP_WARN(get_logger(), "Right turn sound not found: %s", right_sound_path_.c_str());
    right_sound_path_.clear();
  }
  if (!obstacle_sound_path_.empty() && !FileExists(obstacle_sound_path_)) {
    RCLCPP_WARN(get_logger(), "Obstacle sound not found: %s", obstacle_sound_path_.c_str());
    obstacle_sound_path_.clear();
  }

  if (enable_sound_ && !start_sound_path_.empty()) {
    state_ = RunState::kStartSound;
  } else {
    state_ = RunState::kRunning;
  }
  last_time_ = now();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
  marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("car_marker", rclcpp::QoS(10));
  path_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("path_marker", rclcpp::QoS(1).transient_local());
  obstacle_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("obstacle_marker", rclcpp::QoS(1).transient_local());
  detection_marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>("detection_marker", rclcpp::QoS(10));
  obstacle_sub_ = create_subscription<std_msgs::msg::Bool>(
      obstacle_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) { this->OnObstacleFlag(msg); });

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AutoSoundNode::OnTimer, this));
}

void AutoSoundNode::OnObstacleFlag(const std_msgs::msg::Bool::SharedPtr msg) {
  if (!enable_obstacle_sound_) {
    obstacle_flag_ = msg->data;
    return;
  }
  if (msg->data && !obstacle_flag_) {
    obstacle_pending_ = true;
    RCLCPP_INFO(get_logger(), "Obstacle flag received. Pausing.");
  }
  obstacle_flag_ = msg->data;
}

void AutoSoundNode::StartSoundAsync(const std::string& sound_path, const char* label) {
  if (!enable_sound_ || sound_path.empty() || sound_future_.valid()) {
    return;
  }
  sound_label_ = label;
  const std::string command = audio_player_ + " " + ShellQuote(sound_path);
  const auto logger = get_logger();
  sound_future_ = std::async(std::launch::async, [logger, command, sound_path, label]() {
    RCLCPP_INFO(logger, "Playing %s: %s", label, sound_path.c_str());
    const int ret = std::system(command.c_str());
    if (ret != 0) {
      RCLCPP_WARN(logger, "Sound command failed (%s). Return code: %d", label, ret);
    }
    return ret;
  });
}

bool AutoSoundNode::IsSoundFinished() {
  if (!sound_future_.valid()) {
    return true;
  }
  if (sound_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    return false;
  }
  sound_future_.get();
  sound_future_ = std::future<int>();
  sound_label_.clear();
  return true;
}

void AutoSoundNode::PublishPathMarker() {
  const auto& path = component_.path();
  if (path.size() < 2) {
    return;
  }

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = now();
  marker.ns = "path";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = path_width_;
  marker.color.r = path_color_[0];
  marker.color.g = path_color_[1];
  marker.color.b = path_color_[2];
  marker.color.a = path_color_[3];

  marker.points.reserve(path.size());
  for (const auto& wp : path) {
    geometry_msgs::msg::Point p;
    p.x = wp.x;
    p.y = wp.y;
    p.z = 0.0;
    marker.points.push_back(p);
  }

  path_marker_pub_->publish(marker);
}

void AutoSoundNode::PublishObstacleMarker() {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = now();
  marker.ns = "obstacle";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = obstacle_x_;
  marker.pose.position.y = obstacle_y_;
  marker.pose.position.z = obstacle_height_ * 0.5;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = obstacle_radius_ * 2.0;
  marker.scale.y = obstacle_radius_ * 2.0;
  marker.scale.z = obstacle_height_;
  marker.color.r = obstacle_color_[0];
  marker.color.g = obstacle_color_[1];
  marker.color.b = obstacle_color_[2];
  marker.color.a = obstacle_color_[3];
  obstacle_marker_pub_->publish(marker);
}

void AutoSoundNode::PublishDetectionMarker() {
  if (detection_radius_ <= 0.0 || detection_line_width_ <= 0.0) {
    return;
  }

  const Pose2D& pose = component_.pose();
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id_;
  marker.header.stamp = now();
  marker.ns = "detection";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = detection_line_width_;
  marker.color.r = detection_color_[0];
  marker.color.g = detection_color_[1];
  marker.color.b = detection_color_[2];
  marker.color.a = detection_color_[3];

  constexpr int kSegments = 64;
  marker.points.reserve(kSegments + 1);
  for (int i = 0; i <= kSegments; ++i) {
    const double angle = (2.0 * kPi * i) / kSegments;
    geometry_msgs::msg::Point p;
    p.x = pose.x + detection_radius_ * std::cos(angle);
    p.y = pose.y + detection_radius_ * std::sin(angle);
    p.z = 0.01;
    marker.points.push_back(p);
  }

  detection_marker_pub_->publish(marker);
}

void AutoSoundNode::OnTimer() {
  const auto now_time = now();
  double dt = (now_time - last_time_).seconds();
  last_time_ = now_time;

  PublishPathMarker();
  PublishObstacleMarker();
  PublishDetectionMarker();

  IsSoundFinished();

  if (obstacle_auto_detect_ && state_ == RunState::kRunning) {
    const Pose2D& pose = component_.pose();
    const double dist = std::hypot(obstacle_x_ - pose.x, obstacle_y_ - pose.y);
    const double threshold = detection_radius_ + obstacle_radius_;
    if (dist <= threshold && !obstacle_flag_) {
      obstacle_pending_ = true;
      obstacle_flag_ = true;
      RCLCPP_INFO(get_logger(), "Obstacle detected (auto). Pausing.");
    } else if (dist > threshold) {
      obstacle_flag_ = false;
    }
  }

  if (state_ == RunState::kStartSound) {
    if (!enable_sound_ || start_sound_path_.empty()) {
      state_ = RunState::kRunning;
    } else if (!sound_future_.valid()) {
      if (!start_sound_started_) {
        StartSoundAsync(start_sound_path_, "start");
        start_sound_started_ = true;
      } else {
        state_ = RunState::kRunning;
      }
    }
  } else if (state_ == RunState::kGoalSound) {
    if (!enable_sound_ || goal_sound_path_.empty()) {
      state_ = RunState::kStopped;
    } else if (!sound_future_.valid()) {
      if (!goal_sound_started_) {
        StartSoundAsync(goal_sound_path_, "goal");
        goal_sound_started_ = true;
      } else {
        state_ = RunState::kStopped;
      }
    }
  } else if (state_ == RunState::kObstacleSound) {
    if (!enable_sound_ || obstacle_sound_path_.empty()) {
      state_ = RunState::kRunning;
    } else if (!sound_future_.valid()) {
      if (!obstacle_sound_started_) {
        StartSoundAsync(obstacle_sound_path_, "obstacle");
        obstacle_sound_started_ = true;
      } else {
        state_ = RunState::kRunning;
      }
    }
  }

  if (state_ == RunState::kRunning && obstacle_pending_) {
    state_ = RunState::kObstacleSound;
    obstacle_sound_started_ = false;
    obstacle_pending_ = false;
  }

  const bool running = (state_ == RunState::kRunning);
  const Pose2D pose = component_.Step(dt, running);

  if (running && enable_sound_ && enable_turn_sound_) {
    const auto& path = component_.path();
    const std::size_t idx = component_.current_waypoint_index();
    if (!sound_future_.valid() && idx > 0 && idx + 1 < path.size() && idx != last_turn_index_) {
      const auto& wp = path[idx];
      const double dist = std::hypot(wp.x - pose.x, wp.y - pose.y);
      if (dist <= turn_announce_distance_) {
        const auto dir = ComputeTurnDirection(path, idx, turn_min_angle_rad_);
        if (dir == TurnDirection::kLeft && !left_sound_path_.empty()) {
          StartSoundAsync(left_sound_path_, "left");
          last_turn_index_ = idx;
        } else if (dir == TurnDirection::kRight && !right_sound_path_.empty()) {
          StartSoundAsync(right_sound_path_, "right");
          last_turn_index_ = idx;
        } else if (dir != TurnDirection::kStraight) {
          last_turn_index_ = idx;
        }
      }
    }
  }

  if (running && component_.goal_reached()) {
    if (enable_sound_ && !goal_sound_path_.empty()) {
      state_ = RunState::kGoalSound;
      goal_sound_started_ = false;
    } else {
      state_ = RunState::kStopped;
    }
    RCLCPP_INFO(get_logger(), "Goal reached.");
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = now_time;
  tf_msg.header.frame_id = frame_id_;
  tf_msg.child_frame_id = child_frame_id_;
  tf_msg.transform.translation.x = pose.x;
  tf_msg.transform.translation.y = pose.y;
  tf_msg.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose.yaw);
  tf_msg.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(tf_msg);

  nav_msgs::msg::Odometry odom;
  odom.header = tf_msg.header;
  odom.child_frame_id = child_frame_id_;
  odom.pose.pose.position.x = pose.x;
  odom.pose.pose.position.y = pose.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf_msg.transform.rotation;
  odom.twist.twist.linear.x = component_.params().speed_mps;
  odom_pub_->publish(odom);

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = child_frame_id_;
  marker.header.stamp = now_time;
  marker.ns = "car";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = car_length_;
  marker.scale.y = car_width_;
  marker.scale.z = car_height_;
  marker.color.r = car_color_[0];
  marker.color.g = car_color_[1];
  marker.color.b = car_color_[2];
  marker.color.a = car_color_[3];
  marker.lifetime = rclcpp::Duration::from_seconds(0.0);
  marker_pub_->publish(marker);
}

}  // namespace auto_sound
