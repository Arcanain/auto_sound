#include <rclcpp/rclcpp.hpp>

#include "auto_sound/auto_sound_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<auto_sound::AutoSoundNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
