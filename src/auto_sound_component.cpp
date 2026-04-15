#include "auto_sound/auto_sound_component.hpp"

#include <algorithm>
#include <cmath>

namespace auto_sound {

namespace {

double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace

ObstacleSoundTrigger::ObstacleSoundTrigger() { Reset(Params{}); }

ObstacleSoundTrigger::ObstacleSoundTrigger(const Params& params) { Reset(params); }

void ObstacleSoundTrigger::Reset(const Params& params) {
  params_ = params;
  if (!std::isfinite(params_.cooldown_sec) || params_.cooldown_sec < 0.0) {
    params_.cooldown_sec = 0.0;
  }
  last_obstacle_detected_ = false;
  has_last_trigger_time_ = false;
  last_trigger_time_sec_ = 0.0;
}

bool ObstacleSoundTrigger::ShouldTrigger(bool obstacle_detected, double now_sec) {
  if (!std::isfinite(now_sec)) {
    now_sec = 0.0;
  }

  if (!obstacle_detected) {
    last_obstacle_detected_ = false;
    return false;
  }

  if (last_obstacle_detected_) {
    return false;
  }
  last_obstacle_detected_ = true;

  if (has_last_trigger_time_ && now_sec >= last_trigger_time_sec_ &&
      (now_sec - last_trigger_time_sec_) < params_.cooldown_sec) {
    return false;
  }

  last_trigger_time_sec_ = now_sec;
  has_last_trigger_time_ = true;
  return true;
}

AutoSoundComponent::AutoSoundComponent() { Reset(Params{}); }

AutoSoundComponent::AutoSoundComponent(const Params& params) { Reset(params); }

void AutoSoundComponent::Reset(const Params& params) {
  params_ = params;
  pose_.x = params_.start_x;
  pose_.y = params_.start_y;
  pose_.yaw = params_.start_yaw;
  waypoint_index_ = 0;
  goal_reached_ = false;
}

void AutoSoundComponent::SetPath(const std::vector<Waypoint>& path) {
  path_ = path;
  waypoint_index_ = 0;
  goal_reached_ = path_.empty();
}

Pose2D AutoSoundComponent::Step(double dt_sec, bool running) {
  if (!std::isfinite(dt_sec) || dt_sec < 0.0) {
    dt_sec = 0.0;
  }

  if (!running) {
    return pose_;
  }

  double desired_yaw_path = pose_.yaw;

  if (!path_.empty()) {
    if (goal_reached_) {
      return pose_;
    }

    if (waypoint_index_ >= path_.size()) {
      goal_reached_ = true;
      return pose_;
    }

    Waypoint target = path_[waypoint_index_];
    double dx = target.x - pose_.x;
    double dy = target.y - pose_.y;
    double dist = std::hypot(dx, dy);

    while (dist <= params_.waypoint_tolerance && waypoint_index_ + 1 < path_.size()) {
      waypoint_index_++;
      target = path_[waypoint_index_];
      dx = target.x - pose_.x;
      dy = target.y - pose_.y;
      dist = std::hypot(dx, dy);
    }

    if (dist <= params_.waypoint_tolerance && waypoint_index_ + 1 >= path_.size()) {
      goal_reached_ = true;
      return pose_;
    }

    desired_yaw_path = std::atan2(dy, dx);
  }

  const double desired_yaw = desired_yaw_path;

  const double yaw_error = NormalizeAngle(desired_yaw - pose_.yaw);
  const double max_delta = params_.max_yaw_rate * dt_sec;
  const double yaw_step = std::clamp(yaw_error, -max_delta, max_delta);

  pose_.yaw = NormalizeAngle(pose_.yaw + yaw_step);

  const double travel = params_.speed_mps * dt_sec;
  pose_.x += travel * std::cos(pose_.yaw);
  pose_.y += travel * std::sin(pose_.yaw);

  return pose_;
}

}  // namespace auto_sound
