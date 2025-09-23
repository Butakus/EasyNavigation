// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Defines data structures and utilities for representing and processing sensor perceptions.
/// Includes conversion between ROS messages and PCL point clouds, fusion of data, and tools for creating filtered views.

/// \file
/// \brief Defines data structures and utilities for representing and processing point-based sensor perceptions.
///
/// This file includes:
/// - PointPerception: a concrete class for point cloud data.
/// - PointPerceptionHandler: a handler for LaserScan and PointCloud2 messages.
/// - PointPerceptionsOpsView: an efficient view-based operator class for processing multiple perceptions.
/// - Conversion utilities between ROS messages and PCL point clouds.

#ifndef EASYNAV_COMMON_TYPES__POINTPERCEPTIONS_HPP_
#define EASYNAV_COMMON_TYPES__POINTPERCEPTIONS_HPP_

#include <string>
#include <vector>
#include <optional>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"
#include "pcl/common/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/Perceptions.hpp"

namespace std
{

/// \brief Custom std::hash specialization for std::tuple<int, int, int>.
template<>
struct hash<std::tuple<int, int, int>>
{
  std::size_t operator()(const std::tuple<int, int, int> & key) const
  {
    std::size_t h1 = std::hash<int>()(std::get<0>(key));
    std::size_t h2 = std::hash<int>()(std::get<1>(key));
    std::size_t h3 = std::hash<int>()(std::get<2>(key));
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

}  // namespace std

namespace easynav
{

/// \class PointPerception
/// \brief Concrete perception class for 3D point cloud data.
///
/// This class stores a point cloud of type `pcl::PointCloud<pcl::PointXYZ>`
/// and provides utilities such as preallocation.
class PointPerception : public PerceptionBase
{
public:
  /// \brief The 3D point cloud data associated with this perception.
  pcl::PointCloud<pcl::PointXYZ> data;

  /// \brief Resize the internal point cloud to a fixed number of points.
  /// \param size Number of points to allocate.
  void resize(std::size_t size)
  {
    data.points.resize(size);
  }
};

/// \class PointPerceptionHandler
/// \brief PerceptionHandler implementation for sensors producing point-based data.
///
/// This handler supports both `sensor_msgs::msg::LaserScan` and `sensor_msgs::msg::PointCloud2`.
/// It converts incoming messages into `PointPerception` instances and stores them.
class PointPerceptionHandler : public PerceptionHandler
{
public:
  /// \brief Returns the sensor group handled by this handler: "points".
  std::string group() const override {return "points";}

  /// \brief Creates a new `PointPerception` instance.
  /// \param sensor_id Identifier of the sensor (unused in this handler).
  /// \return Shared pointer to a new PointPerception.
  std::shared_ptr<PerceptionBase> create(const std::string &) override
  {
    return std::make_shared<PointPerception>();
  }

  /// \brief Creates a subscription to LaserScan or PointCloud2 messages and updates the  perception.
  rclcpp::SubscriptionBase::SharedPtr create_subscription(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & topic,
    const std::string & type,
    std::shared_ptr<PerceptionBase> target,
    rclcpp::CallbackGroup::SharedPtr cb_group) override;
};

/// \brief Converts a LaserScan message into a point cloud.
/// \param scan The input laser scan message.
/// \param pc Output point cloud (XYZ format).
void convert(const sensor_msgs::msg::LaserScan & scan, pcl::PointCloud<pcl::PointXYZ> & pc);

/// \brief Converts a PointPerception to a sensor_msgs::msg::PointCloud2 message.
/// \param perception The perception to convert.
/// \return The ROS PointCloud2 message.
sensor_msgs::msg::PointCloud2 perception_to_rosmsg(const PointPerception & perception);

/// \brief Converts a PCL point cloud to a sensor_msgs::msg::PointCloud2 message.
/// \param points The input point cloud.
/// \return The ROS PointCloud2 message.
sensor_msgs::msg::PointCloud2 points_to_rosmsg(const pcl::PointCloud<pcl::PointXYZ> & points);

/// \typedef PointPerceptions
/// \brief Alias for a vector of pointers to PointPerception objects.
using PointPerceptions =
  std::vector<std::shared_ptr<PointPerception>>;

PointPerceptions get_point_perceptions(std::vector<PerceptionPtr> & perceptionptr);

/// \class PointPerceptionsOpsView
/// \brief Provides efficient, non-destructive, chainable operations over a set of point-based perceptions.
///
/// This class allows filtering, downsampling, fusion, and collapsing of multiple point clouds
/// without duplicating memory, by using index-based views.
class PointPerceptionsOpsView
{
public:
  /// \struct VoxelKey
  /// \brief Represents a discrete 3D voxel index for downsampling.
  struct VoxelKey
  {
    int x, y, z;

    /// \brief Equality comparison operator.
    bool operator==(const VoxelKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  /// \struct VoxelKeyHash
  /// \brief Hash function for VoxelKey.
  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & key) const
    {
      std::size_t h1 = std::hash<int>{}(key.x);
      std::size_t h2 = std::hash<int>{}(key.y);
      std::size_t h3 = std::hash<int>{}(key.z);
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

  /// \brief Constructor using a constant reference to a container of perceptions.
  /// \param perceptions Container of perceptions to view.
  explicit PointPerceptionsOpsView(const PointPerceptions & perceptions);

  /// \brief Constructor that takes ownership of the container.
  /// \param perceptions Rvalue container of perceptions.
  explicit PointPerceptionsOpsView(PointPerceptions && perceptions);

  /// \brief Filters all point clouds by spatial bounds.
  /// \param min_bounds Minimum [x, y, z] values (use NAN to disable per axis).
  /// \param max_bounds Maximum [x, y, z] values (use NAN to disable per axis).
  /// \return Reference to self for chaining.
  PointPerceptionsOpsView & filter(
    const std::vector<double> & min_bounds,
    const std::vector<double> & max_bounds);

  /// \brief Downsamples each perception using a voxel grid.
  /// \param resolution Size of the voxel in meters.
  /// \return Reference to self for chaining.
  PointPerceptionsOpsView & downsample(double resolution);

  /// \brief Collapses dimensions to fixed values (e.g., project onto plane).
  /// \param collapse_dims Fixed values for each axis (use NAN to preserve original values).
  /// \return New view with collapsed points.
  std::shared_ptr<PointPerceptionsOpsView> collapse(
    const std::vector<double> & collapse_dims) const;

  /// \brief Retrieves all selected points as a single cloud.
  /// \return Concatenated point cloud.
  pcl::PointCloud<pcl::PointXYZ> as_points() const;

  /// \brief Retrieves the filtered point cloud for a specific perception.
  /// \param idx Index of the perception.
  /// \return Const reference to the filtered point cloud.
  const pcl::PointCloud<pcl::PointXYZ> & as_points(int idx) const;

  /// \brief Fuses all perceptions into one, transforming them to a common frame.
  /// \param target_frame Frame ID to transform all clouds into.
  /// \return New fused PointPerceptionsOpsView.
  std::shared_ptr<PointPerceptionsOpsView> fuse(const std::string & target_frame) const;

  /// \brief Add a new Perception from points
  /// \param points new points to include.
  /// \param frame Frame ID of the points to add.
  /// \param stamp Time stamp of the points to add.
  /// \return New PointPerceptionsOpsView.
  std::shared_ptr<PointPerceptionsOpsView> add(
    const pcl::PointCloud<pcl::PointXYZ> points,
    const std::string & frame,
    rclcpp::Time stamp) const;

  /// \brief Gets a reference to the underlying perceptions container.
  /// \return Constant reference to PointPerceptions.
  const PointPerceptions & get_perceptions() const {return perceptions_;}

private:
  std::optional<PointPerceptions> owned_;                  ///< Owned container if moved in.
  const PointPerceptions & perceptions_;                   ///< Reference to perception container.
  std::vector<pcl::PointIndices> indices_;                 ///< Filtered indices per perception.
};

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__POINTPERCEPTIONS_HPP_
