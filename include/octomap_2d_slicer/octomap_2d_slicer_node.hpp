#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <string>
#include <memory>

namespace octomap_2d_slicer
{

class OctomapSlicerNode : public rclcpp::Node
{
public:
  explicit OctomapSlicerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OctomapSlicerNode() = default;

private:
  // Callbacks
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);

  // Core logic
  bool getDroneZ(double & z_out);
  void buildSlice(
    const octomap::OcTree & tree,
    double drone_z,
    nav_msgs::msg::OccupancyGrid & grid);

  // ROS interfaces
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  std::string drone_frame_;   // e.g. "base_link"
  std::string world_frame_;   // e.g. "map"
  double slice_thickness_;    // half-width of Z slice (metres)
  double unknown_as_free_;    // treat unknown cells as free (0) or unknown (-1)
};

}  // namespace octomap_2d_slicer