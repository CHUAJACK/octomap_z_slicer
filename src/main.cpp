#include "octomap_2d_slicer/octomap_2d_slicer_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<octomap_2d_slicer::OctomapSlicerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}