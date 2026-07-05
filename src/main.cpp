#include <rclcpp/rclcpp.hpp>
#include "lio_sam/lio_sam_node.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<lio_sam::LioSamNode>(options);

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 4); // 4 threads
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}