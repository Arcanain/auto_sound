#pragma once

#include <cstdint>
#include <vector>

namespace auto_sound {

class ObstacleSoundTrigger {
 public:
  struct Params {
    double cooldown_sec{5.0};
  };

  ObstacleSoundTrigger();
  explicit ObstacleSoundTrigger(const Params& params);

  void Reset(const Params& params);
  bool ShouldTrigger(bool obstacle_detected, double now_sec);

  const Params& params() const { return params_; }
  bool has_last_trigger_time() const { return has_last_trigger_time_; }
  double last_trigger_time_sec() const { return last_trigger_time_sec_; }

 private:
  Params params_{};
  bool last_obstacle_detected_{false};
  bool has_last_trigger_time_{false};
  double last_trigger_time_sec_{0.0};
};

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Waypoint {
  double x{0.0};
  double y{0.0};
};

class AutoSoundComponent {
 public:
  struct Params {
    double speed_mps{1.0};
    double start_x{0.0};
    double start_y{0.0};
    double start_yaw{0.0};
    double waypoint_tolerance{0.3};
    double max_yaw_rate{1.0};
  };

  AutoSoundComponent();
  explicit AutoSoundComponent(const Params& params);

  void Reset(const Params& params);
  void SetPath(const std::vector<Waypoint>& path);
  Pose2D Step(double dt_sec, bool running = true);

  const Pose2D& pose() const { return pose_; }
  const Params& params() const { return params_; }
  const std::vector<Waypoint>& path() const { return path_; }
  bool goal_reached() const { return goal_reached_; }
  std::size_t current_waypoint_index() const { return waypoint_index_; }

 private:
  Params params_{};
  Pose2D pose_{};
  std::vector<Waypoint> path_{};
  std::size_t waypoint_index_{0};
  bool goal_reached_{false};
};

}  // namespace auto_sound
