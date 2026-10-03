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

#include <csignal>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <unistd.h>

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

bool IsUrl(const std::string& input) {
  return input.rfind("http://", 0) == 0 || input.rfind("https://", 0) == 0;
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

bool IsMpvCommand(const std::string& command) {
  const std::string name = ExtractCommandName(command);
  const auto slash = name.find_last_of('/');
  return ((slash == std::string::npos) ? name : name.substr(slash + 1)) == "mpv";
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

std::string DetectBackgroundAudioPlayer(bool source_is_url, double volume) {
  const int percent = static_cast<int>(std::lround(std::clamp(volume, 0.0, 100.0)));
  if (CommandExists("mpv")) {
    return "mpv --no-video --really-quiet --loop-file=inf --volume=" +
           std::to_string(percent);
  }
  if (source_is_url && CommandExists("yt-dlp") && CommandExists("ffplay")) {
    return "yt-dlp+ffplay";
  }
  if (CommandExists("cvlc")) {
    const double gain = static_cast<double>(percent) / 100.0;
    return "cvlc --intf dummy --no-video --loop --quiet --gain " +
           std::to_string(gain);
  }
  if (!source_is_url && CommandExists("ffplay")) {
    return "ffplay -nodisp -loglevel error -stream_loop -1 -volume " +
           std::to_string(percent);
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
  obstacle_detected_topic_ =
      declare_parameter<std::string>("obstacle_detected_topic", "obstacle_detected");

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
  enable_background_sound_ =
      declare_parameter<bool>("enable_background_sound", true);
  background_sound_source_ = declare_parameter<std::string>(
      "background_sound_source",
      "https://www.youtube.com/watch?v=nXy8Ns6ywfo&list=RDnXy8Ns6ywfo&start_radio=1");
  background_audio_player_ =
      declare_parameter<std::string>("background_audio_player", "auto");
  background_volume_ = declare_parameter<double>("background_volume", 20.0);
  const auto obstacle_sound =
      declare_parameter<std::string>("obstacle_sound", "obstacle.wav");
  obstacle_sound_cooldown_sec_ =
      declare_parameter<double>("obstacle_sound_cooldown_sec", 5.0);
  plate_number_topic_ =
      declare_parameter<std::string>("plate_number_topic", "plate_number");
  plate_repeat_suppression_sec_ =
      declare_parameter<double>("plate_repeat_suppression_sec", 1800.0);
  const auto plate_prefix_sound =
      declare_parameter<std::string>("plate_prefix_sound", "plate_prefix.mp3");
  const auto plate_suffix_sound =
      declare_parameter<std::string>("plate_suffix_sound", "plate_suffix.mp3");
  const auto digit_sound_directory =
      declare_parameter<std::string>("digit_sound_directory", "number");

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
  if (!std::isfinite(obstacle_sound_cooldown_sec_) || obstacle_sound_cooldown_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(),
                "obstacle_sound_cooldown_sec must be >= 0.0. Using 0.0.");
    obstacle_sound_cooldown_sec_ = 0.0;
  }
  if (!std::isfinite(plate_repeat_suppression_sec_) ||
      plate_repeat_suppression_sec_ < 0.0) {
    RCLCPP_WARN(get_logger(),
                "plate_repeat_suppression_sec must be >= 0.0. Using 1800.0.");
    plate_repeat_suppression_sec_ = 1800.0;
  }
  if (!std::isfinite(background_volume_)) {
    background_volume_ = 20.0;
  }
  background_volume_ = std::clamp(background_volume_, 0.0, 100.0);

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

  background_ipc_socket_ =
      "/tmp/auto_sound_mpv_" + std::to_string(static_cast<long long>(getpid())) +
      ".sock";

  const std::string pkg_share =
      ament_index_cpp::get_package_share_directory("auto_sound");
  if (!IsUrl(background_sound_source_)) {
    background_sound_source_ = ResolveSoundPath(background_sound_source_, pkg_share);
    if (!background_sound_source_.empty() && !FileExists(background_sound_source_)) {
      RCLCPP_WARN(get_logger(), "Background sound not found: %s",
                  background_sound_source_.c_str());
      background_sound_source_.clear();
    }
  }
  if (enable_background_sound_ && !background_sound_source_.empty()) {
    if (background_audio_player_ == "auto") {
      background_audio_player_ = DetectBackgroundAudioPlayer(
          IsUrl(background_sound_source_), background_volume_);
    }
    if (background_audio_player_.empty() ||
        (background_audio_player_ != "yt-dlp+ffplay" &&
         !CommandExists(background_audio_player_))) {
      RCLCPP_WARN(get_logger(),
                  "No background audio player found. Install mpv/VLC, or set "
                  "background_audio_player. Background sound is disabled.");
      enable_background_sound_ = false;
    }
  }
  start_sound_path_ = ResolveSoundPath(start_sound, pkg_share);
  if (!start_sound_path_.empty() && !FileExists(start_sound_path_)) {
    RCLCPP_WARN(get_logger(), "Start sound not found: %s", start_sound_path_.c_str());
    start_sound_path_.clear();
  }
  obstacle_sound_path_ = ResolveSoundPath(obstacle_sound, pkg_share);
  if (!obstacle_sound_path_.empty() && !FileExists(obstacle_sound_path_)) {
    RCLCPP_WARN(get_logger(), "Obstacle sound not found: %s",
                obstacle_sound_path_.c_str());
    obstacle_sound_path_.clear();
  }

  plate_prefix_sound_path_ = ResolveSoundPath(plate_prefix_sound, pkg_share);
  plate_suffix_sound_path_ = ResolveSoundPath(plate_suffix_sound, pkg_share);
  plate_sound_ready_ = FileExists(plate_prefix_sound_path_) &&
                       FileExists(plate_suffix_sound_path_);
  for (std::size_t digit = 0; digit < digit_sound_paths_.size(); ++digit) {
    const std::string digit_sound =
        digit_sound_directory + "/" + std::to_string(digit) + ".mp3";
    digit_sound_paths_[digit] = ResolveSoundPath(digit_sound, pkg_share);
    plate_sound_ready_ = plate_sound_ready_ && FileExists(digit_sound_paths_[digit]);
  }
  if (!plate_sound_ready_) {
    RCLCPP_WARN(
        get_logger(),
        "Plate sounds are incomplete. Add %s, %s and %s/{0..9}.mp3; "
        "plate announcements are disabled.",
        plate_prefix_sound_path_.c_str(), plate_suffix_sound_path_.c_str(),
        ResolveSoundPath(digit_sound_directory, pkg_share).c_str());
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
  ObstacleSoundTrigger::Params obstacle_sound_params;
  obstacle_sound_params.cooldown_sec = obstacle_sound_cooldown_sec_;
  obstacle_sound_trigger_.Reset(obstacle_sound_params);
  pending_obstacle_sound_ = false;
  RCLCPP_INFO(get_logger(),
              "Loaded %zu ordered sound cues (radius: %.2f m).",
              ordered_sound_cues_.size(), ordered_cue_radius_);
  if (enable_sound_ && !obstacle_sound_path_.empty()) {
    RCLCPP_INFO(get_logger(),
                "Obstacle sound subscribed on '%s' with %.1f s cooldown.",
                obstacle_detected_topic_.c_str(), obstacle_sound_cooldown_sec_);
  }

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
  obstacle_detected_sub_ = create_subscription<std_msgs::msg::Bool>(
      obstacle_detected_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) { OnObstacleDetected(msg); });
  plate_number_sub_ = create_subscription<std_msgs::msg::String>(
      plate_number_topic_, rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) { OnPlateNumber(msg); });

  path_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "path_marker", rclcpp::QoS(1).transient_local());
  detection_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "detection_marker", rclcpp::QoS(10));

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&AutoSoundNode::OnTimer, this));
}

AutoSoundNode::~AutoSoundNode() {
  StopBackgroundSound();
  unlink(background_ipc_socket_.c_str());
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

void AutoSoundNode::OnObstacleDetected(const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg) {
    return;
  }

  const bool should_trigger =
      obstacle_sound_trigger_.ShouldTrigger(msg->data, now().seconds());
  if (!should_trigger || !enable_sound_ || obstacle_sound_path_.empty()) {
    return;
  }

  if (pending_obstacle_sound_) {
    RCLCPP_DEBUG(get_logger(),
                 "Obstacle alert already pending. Skipping duplicate queue request.");
    return;
  }

  pending_obstacle_sound_ = true;
  RCLCPP_INFO(get_logger(),
              "Obstacle detected. Queueing obstacle sound (cooldown: %.1f s).",
              obstacle_sound_cooldown_sec_);
}

void AutoSoundNode::OnPlateNumber(const std_msgs::msg::String::SharedPtr msg) {
  if (!msg || !enable_sound_ || !plate_sound_ready_) {
    return;
  }

  const std::string& number = msg->data;
  if (number.size() != 3 ||
      !std::all_of(number.begin(), number.end(),
                   [](unsigned char c) { return std::isdigit(c) != 0; })) {
    return;
  }

  const double now_sec = now().seconds();
  const auto previous = plate_last_queued_sec_.find(number);
  if (previous != plate_last_queued_sec_.end() && now_sec >= previous->second &&
      (now_sec - previous->second) < plate_repeat_suppression_sec_) {
    RCLCPP_DEBUG(get_logger(), "Plate %s is within the repeat suppression period.",
                 number.c_str());
    return;
  }

  plate_sound_queue_.push_back({plate_prefix_sound_path_, "plate_prefix"});
  for (const char digit : number) {
    const std::size_t index = static_cast<std::size_t>(digit - '0');
    plate_sound_queue_.push_back(
        {digit_sound_paths_[index], "plate_digit_" + std::string(1, digit)});
  }
  plate_sound_queue_.push_back({plate_suffix_sound_path_, "plate_suffix"});
  plate_last_queued_sec_[number] = now_sec;
  RCLCPP_INFO(get_logger(),
              "Queued plate announcement for %s (repeat suppression: %.0f sec).",
              number.c_str(), plate_repeat_suppression_sec_);
}

bool AutoSoundNode::StartSoundAsync(const std::string& sound_path,
                                    const std::string& label) {
  if (!enable_sound_ || sound_path.empty() || sound_future_.valid()) {
    return false;
  }

  SetBackgroundMuted(true);
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
  return true;
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
  SetBackgroundMuted(false);
  return true;
}

void AutoSoundNode::StartBackgroundSound() {
  if (!enable_sound_ || !enable_background_sound_ ||
      background_sound_source_.empty() || background_sound_pid_ > 0) {
    return;
  }

  std::string command;
  if (background_audio_player_ == "yt-dlp+ffplay") {
    const int percent = static_cast<int>(std::lround(background_volume_));
    command =
        "while true; do media_url=$(yt-dlp --no-playlist -f bestaudio "
        "--get-url " +
        ShellQuote(background_sound_source_) +
        ") || { sleep 5; continue; }; "
        "ffplay -nodisp -autoexit -loglevel error -volume " +
        std::to_string(percent) +
        " \"$media_url\"; sleep 1; done";
  } else {
    command = background_audio_player_ + " " + ShellQuote(background_sound_source_);
  }
  background_uses_mpv_ = IsMpvCommand(background_audio_player_);
  if (background_uses_mpv_) {
    unlink(background_ipc_socket_.c_str());
    command = background_audio_player_ + " --input-ipc-server=" +
              ShellQuote(background_ipc_socket_) + " " +
              ShellQuote(background_sound_source_);
  }
  const pid_t pid = fork();
  if (pid < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to fork background sound process.");
    next_background_start_sec_ = now().seconds() + 5.0;
    return;
  }
  if (pid == 0) {
    setpgid(0, 0);
    execl("/bin/sh", "sh", "-c", command.c_str(), static_cast<char*>(nullptr));
    _exit(127);
  }

  background_sound_pid_ = pid;
  background_muted_ = false;
  background_suspended_ = false;
  setpgid(pid, pid);
  RCLCPP_INFO(get_logger(), "Background sound started at %.0f%% volume: %s",
              background_volume_, background_sound_source_.c_str());
}

void AutoSoundNode::StopBackgroundSound() {
  if (background_sound_pid_ <= 0) {
    return;
  }
  kill(-background_sound_pid_, SIGTERM);
  kill(background_sound_pid_, SIGTERM);
  waitpid(background_sound_pid_, nullptr, 0);
  background_sound_pid_ = -1;
  background_muted_ = false;
  background_suspended_ = false;
  unlink(background_ipc_socket_.c_str());
}

void AutoSoundNode::SetBackgroundMuted(bool muted) {
  if (background_sound_pid_ <= 0 || background_muted_ == muted) {
    return;
  }

  if (background_suspended_) {
    if (!muted) {
      kill(-background_sound_pid_, SIGCONT);
      background_suspended_ = false;
      background_muted_ = false;
    }
    return;
  }

  if (background_uses_mpv_) {
    const int socket_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (socket_fd >= 0) {
      sockaddr_un address{};
      address.sun_family = AF_UNIX;
      if (background_ipc_socket_.size() < sizeof(address.sun_path)) {
        std::copy(background_ipc_socket_.begin(), background_ipc_socket_.end(),
                  address.sun_path);
        if (connect(socket_fd, reinterpret_cast<sockaddr*>(&address),
                    sizeof(address)) == 0) {
          const int volume = muted ? 0 : static_cast<int>(std::lround(background_volume_));
          const std::string request =
              "{\"command\":[\"set_property\",\"volume\"," +
              std::to_string(volume) + "]}\n";
          const ssize_t sent = send(socket_fd, request.data(), request.size(), MSG_NOSIGNAL);
          close(socket_fd);
          if (sent == static_cast<ssize_t>(request.size())) {
            background_muted_ = muted;
            return;
          }
        } else {
          close(socket_fd);
        }
      } else {
        close(socket_fd);
      }
    }
  }

  // Players without runtime volume control are paused as a safe silent fallback.
  if (muted && kill(-background_sound_pid_, SIGSTOP) == 0) {
    background_suspended_ = true;
    background_muted_ = true;
  }
}

void AutoSoundNode::MaintainBackgroundSound() {
  if (!background_sound_allowed_ || !enable_background_sound_) {
    return;
  }
  if (background_sound_pid_ > 0) {
    int status = 0;
    const pid_t result = waitpid(background_sound_pid_, &status, WNOHANG);
    if (result == 0) {
      return;
    }
    background_sound_pid_ = -1;
    next_background_start_sec_ = now().seconds() + 5.0;
    RCLCPP_WARN(get_logger(), "Background sound stopped; retrying in 5 seconds.");
  }
  if (now().seconds() >= next_background_start_sec_) {
    StartBackgroundSound();
  }
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

  MaintainBackgroundSound();

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
        background_sound_allowed_ = true;
        StartBackgroundSound();
      }
    }
    return;
  }

  if (!background_sound_allowed_) {
    background_sound_allowed_ = true;
    StartBackgroundSound();
  }

  if (pending_obstacle_sound_) {
    if (sound_future_.valid()) {
      return;
    }

    pending_obstacle_sound_ = false;
    if (StartSoundAsync(obstacle_sound_path_, "obstacle")) {
      return;
    }
  }

  if (!plate_sound_queue_.empty()) {
    if (sound_future_.valid()) {
      return;
    }
    const auto clip = plate_sound_queue_.front();
    if (StartSoundAsync(clip.path, clip.label)) {
      plate_sound_queue_.pop_front();
      return;
    }
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
