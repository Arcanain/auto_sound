#include "auto_sound/auto_sound_node.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <future>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace auto_sound {

namespace {

constexpr double kPi = 3.14159265358979323846;

std::string Trim(const std::string& input) {
  const auto begin = std::find_if_not(
      input.begin(), input.end(),
      [](unsigned char ch) { return std::isspace(ch) != 0; });
  const auto end = std::find_if_not(
      input.rbegin(), input.rend(),
      [](unsigned char ch) { return std::isspace(ch) != 0; })
                       .base();
  if (begin >= end) {
    return {};
  }
  return std::string(begin, end);
}

bool FileExists(const std::string& path) {
  std::ifstream file(path);
  return file.good();
}

std::string ShellQuote(const std::string& input) {
  std::string out = "'";
  for (const char c : input) {
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
  if (input.front() == '/') {
    return input;
  }
  if (input.rfind("sounds/", 0) == 0) {
    return pkg_share + "/" + input;
  }
  return pkg_share + "/sounds/" + input;
}

struct OrderedCueConfig {
  double x{0.0};
  double y{0.0};
  std::string sound_name{};
};

std::optional<OrderedCueConfig> ParseOrderedCueConfig(const std::string& input) {
  std::stringstream ss(input);
  std::string x_str;
  std::string y_str;
  std::string sound_name;
  if (!std::getline(ss, x_str, ',') || !std::getline(ss, y_str, ',') ||
      !std::getline(ss, sound_name)) {
    return std::nullopt;
  }

  OrderedCueConfig config;
  try {
    config.x = std::stod(Trim(x_str));
    config.y = std::stod(Trim(y_str));
  } catch (const std::exception&) {
    return std::nullopt;
  }

  config.sound_name = Trim(sound_name);
  if (config.sound_name.empty()) {
    return std::nullopt;
  }
  return config;
}

std::string ExtractSoundLabel(const std::string& sound_name) {
  const auto slash = sound_name.find_last_of('/');
  const std::string base =
      (slash == std::string::npos) ? sound_name : sound_name.substr(slash + 1);
  const auto dot = base.find_last_of('.');
  if (dot == std::string::npos) {
    return base;
  }
  return base.substr(0, dot);
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

}  // namespace

AutoSoundNode::AutoSoundNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("auto_sound_node", options) {
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 30.0);
  path_topic_ = declare_parameter<std::string>("path_topic", "tgt_path");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");

  path_width_ = declare_parameter<double>("path_width", 0.15);
  detection_radius_ = declare_parameter<double>("detection_radius", 1.0);
  detection_line_width_ = declare_parameter<double>("detection_line_width", 0.05);

  const auto path_color = declare_parameter<std::vector<double>>(
      "path_color_rgba", std::vector<double>{1.0, 0.7, 0.1, 1.0});
  const auto detection_color = declare_parameter<std::vector<double>>(
      "detection_color_rgba", std::vector<double>{1.0, 0.95, 0.3, 0.35});
  path_color_ = ToColor(path_color, path_color_);
  detection_color_ = ToColor(detection_color, detection_color_);

  enable_sound_ = declare_parameter<bool>("enable_sound", true);
  audio_player_ = declare_parameter<std::string>("audio_player", "auto");
  const auto start_sound = declare_parameter<std::string>("start_sound", "start.wav");

  ordered_cue_radius_ = declare_parameter<double>("ordered_cue_radius", 1.0);
  const auto ordered_sound_cues = declare_parameter<std::vector<std::string>>(
      "ordered_sound_cues",
      std::vector<std::string>{
          "-7.06949,5.24913,left.wav",
          "-0.56388,86.4047,left.wav",
          "-3.88193,100.235,left.wav",
          "-18.0264,101.467,right.wav",
          "-97.1574,111.221,left.wav",
          "-109.494,43.8416,left.wav",
          "-36.7486,21.0162,left.wav",
          "-31.0053,25.3745,right.wav",
          "-7.96139,24.8385,left.wav",
          "0.256114,32.1872,right.wav",
          "28.2829,30.0202,stop.wav",
          "59.5565,37.3161,goal1.wav",
      });

  if (publish_rate_hz_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "publish_rate_hz must be > 0.0. Using 30.0.");
    publish_rate_hz_ = 30.0;
  }
  if (ordered_cue_radius_ < 0.0) {
    ordered_cue_radius_ = 0.0;
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

  const std::string pkg_share =
      ament_index_cpp::get_package_share_directory("auto_sound");
  start_sound_path_ = ResolveSoundPath(start_sound, pkg_share);
  if (!start_sound_path_.empty() && !FileExists(start_sound_path_)) {
    RCLCPP_WARN(get_logger(), "Start sound not found: %s", start_sound_path_.c_str());
    start_sound_path_.clear();
  }

  ordered_sound_cues_.clear();
  ordered_sound_cues_.reserve(ordered_sound_cues.size());
  for (const auto& cue_text : ordered_sound_cues) {
    const auto cue_config = ParseOrderedCueConfig(cue_text);
    if (!cue_config.has_value()) {
      RCLCPP_WARN(get_logger(),
                  "Invalid ordered cue format: '%s' (expected: x,y,sound.wav)",
                  cue_text.c_str());
      continue;
    }

    const std::string cue_sound_path =
        ResolveSoundPath(cue_config->sound_name, pkg_share);
    if (!FileExists(cue_sound_path)) {
      RCLCPP_WARN(get_logger(), "Ordered cue sound not found: %s",
                  cue_sound_path.c_str());
      continue;
    }

    OrderedSoundCue cue;
    cue.x = cue_config->x;
    cue.y = cue_config->y;
    cue.sound_path = cue_sound_path;
    cue.label = ExtractSoundLabel(cue_config->sound_name);
    ordered_sound_cues_.push_back(cue);
  }

  next_ordered_cue_index_ = 0;
  RCLCPP_INFO(get_logger(),
              "Loaded %zu ordered sound cues (radius: %.2f m).",
              ordered_sound_cues_.size(), ordered_cue_radius_);

  if (enable_sound_ && !start_sound_path_.empty()) {
    state_ = RunState::kStartSound;
  } else {
    state_ = RunState::kRunning;
  }

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { OnOdometry(msg); });

  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, rclcpp::QoS(10),
      [this](const nav_msgs::msg::Path::SharedPtr msg) { OnPath(msg); });

  path_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "path_marker", rclcpp::QoS(1).transient_local());
  detection_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "detection_marker", rclcpp::QoS(10));

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AutoSoundNode::OnTimer, this));
}

void AutoSoundNode::OnOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  odom_received_ = true;
}

void AutoSoundNode::OnPath(const nav_msgs::msg::Path::SharedPtr msg) {
  path_points_.clear();
  path_points_.reserve(msg->poses.size());
  for (const auto& pose : msg->poses) {
    PathPoint p;
    p.x = pose.pose.position.x;
    p.y = pose.pose.position.y;
    path_points_.push_back(p);
  }
  path_received_ = !path_points_.empty();
}

void AutoSoundNode::StartSoundAsync(const std::string& sound_path,
                                    const std::string& label) {
  if (!enable_sound_ || sound_path.empty() || sound_future_.valid()) {
    return;
  }

  sound_label_ = label;
  const std::string command = audio_player_ + " " + ShellQuote(sound_path);
  const auto logger = get_logger();
  sound_future_ = std::async(std::launch::async, [logger, command, sound_path, label]() {
    RCLCPP_INFO(logger, "Playing %s: %s", label.c_str(), sound_path.c_str());
    const int ret = std::system(command.c_str());
    if (ret != 0) {
      RCLCPP_WARN(logger, "Sound command failed (%s). Return code: %d",
                  label.c_str(), ret);
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
  if (path_points_.size() < 2) {
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

  marker.points.reserve(path_points_.size());
  for (const auto& wp : path_points_) {
    geometry_msgs::msg::Point p;
    p.x = wp.x;
    p.y = wp.y;
    p.z = 0.0;
    marker.points.push_back(p);
  }

  path_marker_pub_->publish(marker);
}

void AutoSoundNode::PublishDetectionMarker() {
  if (!odom_received_ || detection_radius_ <= 0.0 || detection_line_width_ <= 0.0) {
    return;
  }

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
    const double angle = (2.0 * kPi * static_cast<double>(i)) /
                         static_cast<double>(kSegments);
    geometry_msgs::msg::Point p;
    p.x = robot_x_ + detection_radius_ * std::cos(angle);
    p.y = robot_y_ + detection_radius_ * std::sin(angle);
    p.z = 0.01;
    marker.points.push_back(p);
  }

  detection_marker_pub_->publish(marker);
}

void AutoSoundNode::OnTimer() {
  IsSoundFinished();

  PublishPathMarker();
  PublishDetectionMarker();

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
    return;
  }

  if (!odom_received_ || !path_received_ || sound_future_.valid()) {
    return;
  }

  if (next_ordered_cue_index_ >= ordered_sound_cues_.size()) {
    return;
  }

  const auto& cue = ordered_sound_cues_[next_ordered_cue_index_];
  const double dist = std::hypot(cue.x - robot_x_, cue.y - robot_y_);
  if (dist <= ordered_cue_radius_) {
    StartSoundAsync(cue.sound_path, cue.label);
    RCLCPP_INFO(get_logger(),
                "Ordered cue %zu/%zu triggered at (%.3f, %.3f): %s",
                next_ordered_cue_index_ + 1,
                ordered_sound_cues_.size(),
                cue.x,
                cue.y,
                cue.label.c_str());
    ++next_ordered_cue_index_;
  }
}

}  // namespace auto_sound
