#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "auto_sound/auto_sound_component.hpp"
#include "test_utils.hpp"

#ifndef TEST_DATA_DIR
#define TEST_DATA_DIR ""
#endif

namespace {

std::string JoinPath(const std::string& base, const std::string& name) {
  if (base.empty()) {
    return name;
  }
  if (base.back() == '/') {
    return base + name;
  }
  return base + "/" + name;
}

}  // namespace

TEST(AutoSoundComponent, StepStraight) {
  const std::string input_path = JoinPath(TEST_DATA_DIR, "step.csv");
  const std::string output_path = JoinPath(TEST_DATA_DIR, "test_results_step.csv");

  const auto rows = auto_sound::test::ReadCsvDoubles(input_path, true);
  ASSERT_FALSE(rows.empty()) << "No test data loaded from " << input_path;

  std::vector<std::string> results;
  results.push_back("dt,speed_mps,start_x,expected_x,actual_x,ok");

  for (const auto& row : rows) {
    ASSERT_GE(row.size(), 4u);
    const double dt = row[0];
    const double speed = row[1];
    const double start_x = row[2];
    const double expected_x = row[3];

    auto_sound::AutoSoundComponent::Params params;
    params.speed_mps = speed;
    params.start_x = start_x;
    auto_sound::AutoSoundComponent component(params);

    const auto pose = component.Step(dt, true);
    const double actual_x = pose.x;

    const bool ok = std::fabs(actual_x - expected_x) < 1e-6;
    results.push_back(std::to_string(dt) + "," + std::to_string(speed) + "," +
                      std::to_string(start_x) + "," + std::to_string(expected_x) +
                      "," + std::to_string(actual_x) + "," + (ok ? "OK" : "NG"));

    EXPECT_NEAR(actual_x, expected_x, 1e-6);
  }

  auto_sound::test::WriteLines(output_path, results);
}

TEST(ObstacleSoundTrigger, SuppressesContinuousTrueAndCooldownRetriggers) {
  auto_sound::ObstacleSoundTrigger::Params params;
  params.cooldown_sec = 5.0;
  auto_sound::ObstacleSoundTrigger trigger(params);

  EXPECT_TRUE(trigger.ShouldTrigger(true, 10.0));
  EXPECT_FALSE(trigger.ShouldTrigger(true, 10.1));

  EXPECT_FALSE(trigger.ShouldTrigger(false, 10.2));
  EXPECT_FALSE(trigger.ShouldTrigger(true, 12.0));
  EXPECT_FALSE(trigger.ShouldTrigger(true, 12.1));

  EXPECT_FALSE(trigger.ShouldTrigger(false, 12.2));
  EXPECT_TRUE(trigger.ShouldTrigger(true, 15.1));
}

TEST(ObstacleSoundTrigger, NegativeCooldownIsClampedToZero) {
  auto_sound::ObstacleSoundTrigger::Params params;
  params.cooldown_sec = -1.0;
  auto_sound::ObstacleSoundTrigger trigger(params);

  EXPECT_TRUE(trigger.ShouldTrigger(true, 1.0));
  EXPECT_FALSE(trigger.ShouldTrigger(true, 1.1));

  EXPECT_FALSE(trigger.ShouldTrigger(false, 1.2));
  EXPECT_TRUE(trigger.ShouldTrigger(true, 1.3));
}
