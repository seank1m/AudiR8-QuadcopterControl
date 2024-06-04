#include "mission.h"
#include "ackerman.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto mission_node = std::make_shared<Mission>();
  auto ack_node = std::make_shared<Ackerman>();
  
  rclcpp::executors::MultiThreadedExecutor executor;

  executor.add_node(mission_node);
  executor.add_node(ack_node);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}