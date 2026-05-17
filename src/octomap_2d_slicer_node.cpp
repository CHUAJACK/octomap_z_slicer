#include "octomap_2d_slicer/octomap_2d_slicer_node.hpp"

#include <tf2/exceptions.h>
#include <cmath>

namespace octomap_2d_slicer
{

OctomapSlicerNode::OctomapSlicerNode(const rclcpp::NodeOptions & options)
: Node("octomap_2d_slicer", options)
{
  // ── Parameters ──────────────────────────────────────────────────────────────
  this->declare_parameter<std::string>("drone_frame",     "base_link");
  this->declare_parameter<std::string>("world_frame",     "map");
  this->declare_parameter<double>     ("slice_thickness", 0.2);   // ±0.2 m around drone Z
  this->declare_parameter<bool>       ("unknown_as_free", false);
  this->declare_parameter<bool>       ("use_sim_time",true);

  drone_frame_     = this->get_parameter("base_link").as_string();
  world_frame_     = this->get_parameter("map").as_string();
  slice_thickness_ = this->get_parameter("slice_thickness").as_double();
  unknown_as_free_ = this->get_parameter("unknown_as_free").as_bool() ? 0.0 : -1.0;

  // ── TF2 ─────────────────────────────────────────────────────────────────────
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  // ── Publisher ────────────────────────────────────────────────────────────────
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "octomap_2d_slice", rclcpp::QoS(1).transient_local());

  // ── Subscriber ───────────────────────────────────────────────────────────────
  octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "octomap_binary", rclcpp::QoS(1).best_effort(),
    std::bind(&OctomapSlicerNode::octomapCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
    "OctomapSlicerNode ready  drone_frame=%s  world_frame=%s  slice_thickness=±%.2f m",
    drone_frame_.c_str(), world_frame_.c_str(), slice_thickness_);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TF lookup: drone Z in world frame
// ─────────────────────────────────────────────────────────────────────────────
bool OctomapSlicerNode::getDroneZ(double & z_out)
{
  try {
    auto tf = tf_buffer_->lookupTransform(
      world_frame_, drone_frame_,
      tf2::TimePointZero,                    // latest available
      tf2::durationFromSec(0.1));            // 100 ms timeout
    z_out = tf.transform.translation.z;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "TF lookup failed (%s → %s): %s",
      world_frame_.c_str(), drone_frame_.c_str(), ex.what());
    return false;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Build a 2-D OccupancyGrid from all OctoTree leaves inside the Z slice
// ─────────────────────────────────────────────────────────────────────────────
void OctomapSlicerNode::buildSlice(
  const octomap::OcTree & tree,
  double drone_z,
  nav_msgs::msg::OccupancyGrid & grid)
{
  const double res  = tree.getResolution();
  const double zMin = drone_z - slice_thickness_;
  const double zMax = drone_z + slice_thickness_;

  // Compute world bounding box
  double x_min, y_min, z_min_tree, x_max, y_max, z_max_tree;
  tree.getMetricMin(x_min, y_min, z_min_tree);
  tree.getMetricMax(x_max, y_max, z_max_tree);

  // Grid dimensions
  const int width  = static_cast<int>(std::ceil((x_max - x_min) / res));
  const int height = static_cast<int>(std::ceil((y_max - y_min) / res));

  // Fill header & metadata
  grid.info.resolution      = static_cast<float>(res);
  grid.info.width           = static_cast<uint32_t>(width);
  grid.info.height          = static_cast<uint32_t>(height);
  grid.info.origin.position.x = x_min;
  grid.info.origin.position.y = y_min;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  // Default: unknown (-1)
  grid.data.assign(static_cast<size_t>(width * height),
    static_cast<int8_t>(unknown_as_free_));

  // Iterate over every leaf node of the tree
  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it)
  {
    const double z = it.getZ();
    if (z < zMin || z > zMax) {
      continue;   // outside our altitude slice
    }

    // Map world coords → grid cell indices
    const int gx = static_cast<int>(std::floor((it.getX() - x_min) / res));
    const int gy = static_cast<int>(std::floor((it.getY() - y_min) / res));

    if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
      continue;
    }

    const size_t idx = static_cast<size_t>(gy * width + gx);

    if (tree.isNodeOccupied(*it)) {
      // Occupied – mark 100; once occupied, don't overwrite with free
      grid.data[idx] = 100;
    } else {
      // Free – only write if not already marked occupied
      if (grid.data[idx] != 100) {
        grid.data[idx] = 0;
      }
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main callback
// ─────────────────────────────────────────────────────────────────────────────
void OctomapSlicerNode::octomapCallback(
  const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  // 1. Get drone altitude from TF
  double drone_z = 0.0;
  if (!getDroneZ(drone_z)) {
    return;   // skip this map update if TF not ready
  }

  RCLCPP_DEBUG(this->get_logger(),
    "Processing octomap (seq stamp %.3f s)  drone_z=%.3f m",
    rclcpp::Time(msg->header.stamp).seconds(), drone_z);

  // 2. Deserialise binary OctoMap message → OcTree
  octomap::AbstractOcTree * abstract_tree =
    octomap_msgs::binaryMsgToMap(*msg);

  if (!abstract_tree) {
    RCLCPP_ERROR(this->get_logger(), "Failed to deserialise octomap_binary message");
    return;
  }

  auto * tree = dynamic_cast<octomap::OcTree *>(abstract_tree);
  if (!tree) {
    RCLCPP_ERROR(this->get_logger(),
      "Octomap is not an OcTree (type: %s)", abstract_tree->getTreeType().c_str());
    delete abstract_tree;
    return;
  }

  // 3. Build 2-D slice
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp    = msg->header.stamp;
  grid.header.frame_id = world_frame_;

  buildSlice(*tree, drone_z, grid);
  delete tree;

  // 4. Publish
  map_pub_->publish(grid);

  RCLCPP_INFO_ONCE(this->get_logger(), "First 2-D slice published.");
  RCLCPP_DEBUG(this->get_logger(),
    "Published 2-D slice: %u×%u cells at drone_z=%.2f m (slice ±%.2f m)",
    grid.info.width, grid.info.height, drone_z, slice_thickness_);
}

}  // namespace octomap_2d_slicer