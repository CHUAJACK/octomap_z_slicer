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
  this->declare_parameter<double>     ("slice_thickness", 0.5);
  this->declare_parameter<bool>       ("unknown_as_free", false);
  this->declare_parameter<int>        ("publish_rate_ms", 100);

  drone_frame_     = this->get_parameter("drone_frame").as_string();
  world_frame_     = this->get_parameter("world_frame").as_string();
  slice_thickness_ = this->get_parameter("slice_thickness").as_double();
  unknown_as_free_ = this->get_parameter("unknown_as_free").as_bool() ? 0.0 : -1.0;
  int publish_rate_ms = this->get_parameter("publish_rate_ms").as_int();

  // ── TF2 ─────────────────────────────────────────────────────────────────────
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  // ── Publisher ────────────────────────────────────────────────────────────────
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "octomap_2d_slice", rclcpp::QoS(1).transient_local());

  // ── Publish timer ────────────────────────────────────────────────────────────
  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(publish_rate_ms),
    std::bind(&OctomapSlicerNode::publishTimer, this));

  // ── Subscriber ───────────────────────────────────────────────────────────────
  octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "octomap_binary", rclcpp::QoS(1).best_effort(),
    std::bind(&OctomapSlicerNode::octomapCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
    "OctomapSlicerNode ready  drone_frame=%s  world_frame=%s  "
    "slice_thickness=±%.2f m  publish_rate=%d ms",
    drone_frame_.c_str(), world_frame_.c_str(),
    slice_thickness_, publish_rate_ms);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Timer callback — publishes persistent grid at a fixed rate
// ─────────────────────────────────────────────────────────────────────────────
void OctomapSlicerNode::publishTimer()
{
  if (!persistent_grid_initialized_) return;
  persistent_grid_.header.stamp = this->get_clock()->now();
  map_pub_->publish(persistent_grid_);
}

// ─────────────────────────────────────────────────────────────────────────────
//  TF lookup: drone Z in world frame
// ─────────────────────────────────────────────────────────────────────────────
bool OctomapSlicerNode::getDroneZ(double & z_out)
{
  try {
    auto tf = tf_buffer_->lookupTransform(
      world_frame_, drone_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(0.1));
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
//  Remap persistent grid into a new coordinate frame when octomap grows
// ─────────────────────────────────────────────────────────────────────────────
void OctomapSlicerNode::remapPersistentGrid(
  const nav_msgs::msg::OccupancyGrid & new_frame)
{
  const double res = new_frame.info.resolution;
  const int new_w  = static_cast<int>(new_frame.info.width);
  const int new_h  = static_cast<int>(new_frame.info.height);

  std::vector<int8_t> remapped(
    static_cast<size_t>(new_w * new_h),
    static_cast<int8_t>(unknown_as_free_));

  // Offset in cells between old origin and new origin
  const int ox = static_cast<int>(std::round(
    (persistent_grid_.info.origin.position.x -
     new_frame.info.origin.position.x) / res));
  const int oy = static_cast<int>(std::round(
    (persistent_grid_.info.origin.position.y -
     new_frame.info.origin.position.y) / res));

  const int old_w = static_cast<int>(persistent_grid_.info.width);
  const int old_h = static_cast<int>(persistent_grid_.info.height);

  for (int gy = 0; gy < old_h; ++gy) {
    for (int gx = 0; gx < old_w; ++gx) {
      const int new_gx = gx + ox;
      const int new_gy = gy + oy;
      if (new_gx < 0 || new_gx >= new_w) continue;
      if (new_gy < 0 || new_gy >= new_h) continue;
      remapped[static_cast<size_t>(new_gy * new_w + new_gx)] =
        persistent_grid_.data[static_cast<size_t>(gy * old_w + gx)];
    }
  }

  persistent_grid_      = new_frame;
  persistent_grid_.data = remapped;
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
  grid.info.resolution        = static_cast<float>(res);
  grid.info.width             = static_cast<uint32_t>(width);
  grid.info.height            = static_cast<uint32_t>(height);
  grid.info.origin.position.x = x_min;
  grid.info.origin.position.y = y_min;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  // Default: unknown (-1) or free (0) depending on parameter
  grid.data.assign(static_cast<size_t>(width * height),
    static_cast<int8_t>(unknown_as_free_));

  // Iterate over every leaf node of the tree
  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it)
  {
    const double z = it.getZ();
    if (z < zMin || z > zMax) continue;

    // Get the actual size of this leaf voxel (octree nodes vary in size)
    const double node_size = it.getSize();
    const double half      = node_size / 2.0;

    // World-space bounding box of this voxel in XY
    const double wx_min = it.getX() - half;
    const double wx_max = it.getX() + half;
    const double wy_min = it.getY() - half;
    const double wy_max = it.getY() + half;

    // Grid-cell range this voxel covers
    const int gx_min = static_cast<int>(std::floor((wx_min - x_min) / res));
    const int gx_max = static_cast<int>(std::floor((wx_max - x_min) / res));
    const int gy_min = static_cast<int>(std::floor((wy_min - y_min) / res));
    const int gy_max = static_cast<int>(std::floor((wy_max - y_min) / res));

    const bool occupied = tree.isNodeOccupied(*it);

    // Stamp every grid cell this voxel covers
    for (int gy = gy_min; gy <= gy_max; ++gy) {
      if (gy < 0 || gy >= height) continue;
      for (int gx = gx_min; gx <= gx_max; ++gx) {
        if (gx < 0 || gx >= width) continue;

        const size_t idx = static_cast<size_t>(gy * width + gx);
        if (occupied) {
          grid.data[idx] = 100;
        } else if (grid.data[idx] != 100) {
          grid.data[idx] = 0;
        }
      }
    }
  }
}

void OctomapSlicerNode::inflateFreeSpace(
  nav_msgs::msg::OccupancyGrid & grid, int passes)
{
  const int width  = static_cast<int>(grid.info.width);
  const int height = static_cast<int>(grid.info.height);

  for (int pass = 0; pass < passes; ++pass) {
    std::vector<int8_t> output = grid.data;  // copy each pass

    for (int gy = 1; gy < height - 1; ++gy) {
      for (int gx = 1; gx < width - 1; ++gx) {
        const size_t idx = static_cast<size_t>(gy * width + gx);
        if (grid.data[idx] != -1) continue;  // only expand into unknowns

        // Count free and occupied neighbours in 3x3 window
        int free_count = 0;
        bool has_occupied_neighbour = false;
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dy == 0 && dx == 0) continue;
            const size_t nidx = static_cast<size_t>((gy+dy) * width + (gx+dx));
            if (grid.data[nidx] == 0)   ++free_count;
            if (grid.data[nidx] == 100) has_occupied_neighbour = true;
          }
        }

        // Only mark free if it has free neighbours but no occupied ones
        // This prevents bleeding through walls
        if (free_count >= 1 && !has_occupied_neighbour) {
          output[idx] = 0;
        }
      }
    }
    grid.data = output;
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
    return;
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

  // 3. Build current 2-D slice
  nav_msgs::msg::OccupancyGrid current_grid;
  current_grid.header.stamp    = msg->header.stamp;
  current_grid.header.frame_id = world_frame_;
  buildSlice(*tree, drone_z, current_grid);
  delete tree;

  // 4. Merge into persistent grid
  if (!persistent_grid_initialized_) {
    persistent_grid_             = current_grid;
    persistent_grid_initialized_ = true;
  } else {
    // If the octomap grew and changed dimensions, remap persistent grid first
    if (current_grid.info.width  != persistent_grid_.info.width  ||
        current_grid.info.height != persistent_grid_.info.height ||
        current_grid.info.origin.position.x != persistent_grid_.info.origin.position.x ||
        current_grid.info.origin.position.y != persistent_grid_.info.origin.position.y)
    {
      remapPersistentGrid(current_grid);
    }

    // Merge rule:
    // - Occupied in current slice → mark occupied (trust current sensor)
    // - Free in current slice     → mark free (even if previously occupied)
    // - Unknown in current slice  → if previously occupied, clear to free
    //                               otherwise keep previous value
    for (size_t i = 0; i < current_grid.data.size(); ++i) {
      const int8_t cur  = current_grid.data[i];
      const int8_t prev = persistent_grid_.data[i];

      if (cur == 100) {
        persistent_grid_.data[i] = 100;   // occupied — trust current slice
      } else if (cur == 0) {
        persistent_grid_.data[i] = 0;     // free — trust current slice
      } else {
        // Current slice has no opinion (unknown)
        if (prev == 100) {
          persistent_grid_.data[i] = 0;   // was obstacle, no longer in slice → clear
        }
        // prev == 0 or -1: keep as-is
      }
    }
    //inflateFreeSpace(persistent_grid_, 1);  // adjust pass count as needed
    persistent_grid_.header.stamp = msg->header.stamp;
  }

  RCLCPP_INFO_ONCE(this->get_logger(), "First 2-D slice published.");
  RCLCPP_DEBUG(this->get_logger(),
    "Published 2-D slice: %u×%u cells at drone_z=%.2f m (slice ±%.2f m)",
    persistent_grid_.info.width, persistent_grid_.info.height,
    drone_z, slice_thickness_);
}

}  // namespace octomap_2d_slicer