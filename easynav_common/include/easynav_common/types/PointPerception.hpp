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
/// \brief Defines data structures and utilities for representing and processing point-based sensor perceptions.
///
/// This header provides:
/// - \c PointPerception: a concrete class for point cloud data.
/// - \c PointPerceptionHandler: a handler for \c sensor_msgs::msg::LaserScan and
///   \c sensor_msgs::msg::PointCloud2 messages.
/// - \c PointPerceptionsOpsView: an efficient view-based operator class for processing multiple perceptions
///   (filtering, downsampling, fusion, and collapsing) without duplicating memory.
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

/// \brief Custom hash specialization for \c std::tuple<int,int,int>.
template<>
struct hash<std::tuple<int, int, int>>
{
  /// \brief Computes the hash value.
  /// \param key Tuple \c (x,y,z).
  /// \return Hash value for the tuple.
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
/// Stores a point cloud of type \c pcl::PointCloud<pcl::PointXYZ> and the common metadata inherited
/// from \ref PerceptionBase.
class PointPerception : public PerceptionBase
{
public:
  /// \brief Group identifier for point perceptions.
  static constexpr std::string_view default_group_ = "points";

  /// \brief Checks if a ROS message type is supported by this perception.
  /// \param t Fully-qualified ROS 2 message type name (e.g., \c "sensor_msgs/msg/PointCloud2").
  /// \return \c true if \p t is \c sensor_msgs/msg/LaserScan or \c sensor_msgs/msg/PointCloud2, otherwise \c false.
  static inline bool supports_msg_type(std::string_view t)
  {
    return t == "sensor_msgs/msg/LaserScan" ||
           t == "sensor_msgs/msg/PointCloud2";
  }

  /// \brief The 3D point cloud data associated with this perception.
  pcl::PointCloud<pcl::PointXYZ> data;

  /// \brief Resizes the internal point cloud storage.
  /// \param size Number of points to allocate in \ref data.
  void resize(std::size_t size)
  {
    data.points.resize(size);
  }
};

/// \class PointPerceptionHandler
/// \brief \ref PerceptionHandler implementation for sensors producing point-based data.
///
/// Supports both \c sensor_msgs::msg::LaserScan and \c sensor_msgs::msg::PointCloud2, converting
/// incoming messages into \ref PointPerception instances.
class PointPerceptionHandler : public PerceptionHandler
{
public:
  /// \brief Returns the sensor group handled by this handler.
  /// \return The string literal \c "points".
  std::string group() const override {return "points";}

  /// \brief Creates a new \ref PointPerception instance.
  /// \param sensor_id Identifier of the sensor (currently unused, kept for future extensions).
  /// \return Shared pointer to a new \ref PointPerception.
  std::shared_ptr<PerceptionBase> create(const std::string &) override
  {
    return std::make_shared<PointPerception>();
  }

  /// \brief Creates a subscription to \c LaserScan or \c PointCloud2 messages and updates the perception.
  ///
  /// The created subscription decodes incoming messages from \p topic according to \p type, converts them
  /// into a point cloud, and writes the result into \p target (including metadata such as \c stamp and \c frame_id).
  ///
  /// \param node Lifecycle node used to create the subscription.
  /// \param topic Topic name to subscribe to.
  /// \param type ROS message type name. Must be either \c "sensor_msgs/msg/LaserScan"
  ///             or \c "sensor_msgs/msg/PointCloud2".
  /// \param target Shared pointer to the perception instance to be updated.
  /// \param cb_group Callback group for the subscription callback (executor-level concurrency control).
  /// \return Shared pointer to the created subscription.
  rclcpp::SubscriptionBase::SharedPtr create_subscription(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & topic,
    const std::string & type,
    std::shared_ptr<PerceptionBase> target,
    rclcpp::CallbackGroup::SharedPtr cb_group) override;
};

/// \brief Converts a \c LaserScan message into a point cloud.
/// \param scan Input laser scan message.
/// \param pc Output point cloud (XYZ).
void convert(const sensor_msgs::msg::LaserScan & scan, pcl::PointCloud<pcl::PointXYZ> & pc);

/// \brief Converts a \ref PointPerception into a \c sensor_msgs::msg::PointCloud2 message.
/// \param perception Perception to convert.
/// \return The resulting ROS \c PointCloud2 message.
sensor_msgs::msg::PointCloud2 perception_to_rosmsg(const PointPerception & perception);

/// \brief Converts a PCL point cloud into a \c sensor_msgs::msg::PointCloud2 message.
/// \param points Input point cloud.
/// \return The resulting ROS \c PointCloud2 message.
sensor_msgs::msg::PointCloud2 points_to_rosmsg(const pcl::PointCloud<pcl::PointXYZ> & points);

/// \typedef PointPerceptions
/// \brief Alias for a vector of shared pointers to \ref PointPerception objects.
using PointPerceptions =
  std::vector<std::shared_ptr<PointPerception>>;

/// \brief Extracts all \ref PointPerception objects from a heterogeneous collection.
/// \param perceptionptr Vector of \ref PerceptionPtr entries possibly holding mixed perception types.
/// \return A vector with the subset of perceptions that are \ref PointPerception.
PointPerceptions get_point_perceptions(std::vector<PerceptionPtr> & perceptionptr);

/// \class PointPerceptionsOpsView
/// \brief Provides efficient, non-destructive, chainable operations over a set of point-based perceptions.
///
/// This view enables filtering, downsampling, fusion, and dimensional collapsing across multiple point clouds
/// without duplicating memory, by keeping index-based selections per perception.
class PointPerceptionsOpsView
{
public:
  /// \struct VoxelKey
  /// \brief Discrete 3D voxel index used for downsampling.
  struct VoxelKey
  {
    /// \brief Discrete coordinates (voxel indices).
    int x, y, z;

    /// \brief Equality comparison.
    /// \param other Voxel key to compare against.
    /// \return \c true if all components are equal, otherwise \c false.
    bool operator==(const VoxelKey & other) const
    {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  /// \struct VoxelKeyHash
  /// \brief Hash functor for \ref VoxelKey.
  struct VoxelKeyHash
  {
    /// \brief Computes the hash value.
    /// \param key Voxel key.
    /// \return Hash value for \p key.
    std::size_t operator()(const VoxelKey & key) const
    {
      std::size_t h1 = std::hash<int>{}(key.x);
      std::size_t h2 = std::hash<int>{}(key.y);
      std::size_t h3 = std::hash<int>{}(key.z);
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

  /// \brief Constructs a view over an external container of perceptions.
  /// \param perceptions Constant reference to the source container.
  explicit PointPerceptionsOpsView(const PointPerceptions & perceptions);


  /// \brief Constructs a view from a single PointPerception instance.
  ///
  /// Creates an internal container owning the given perception and
  /// initializes the index set for all its points.
  ///
  /// \param perception The PointPerception to wrap in the view.
  explicit PointPerceptionsOpsView(const PointPerception & perception);


  /// \brief Constructs a view taking ownership of the container.
  /// \param perceptions Rvalue container of perceptions to be owned by the view.
  explicit PointPerceptionsOpsView(PointPerceptions && perceptions);

  /// \brief Filters all point clouds by axis-aligned bounds.
  ///
  /// Components set to \c NaN in \p min_bounds or \p max_bounds leave the corresponding axis unbounded.
  ///
  /// \param min_bounds Minimum \c [x,y,z] values (use \c NaN to disable per axis).
  /// \param max_bounds Maximum \c [x,y,z] values (use \c NaN to disable per axis).
  /// \return Reference to \c *this to allow chaining.
  PointPerceptionsOpsView & filter(
    const std::vector<double> & min_bounds,
    const std::vector<double> & max_bounds);

  /// \brief Downsamples each perception using a voxel grid.
  /// \param resolution Voxel size in meters.
  /// \return Reference to \c *this to allow chaining.
  PointPerceptionsOpsView & downsample(double resolution);

  /// \brief Collapses dimensions to fixed values (e.g., projection onto a plane).
  ///
  /// Components set to \c NaN in \p collapse_dims keep the original values.
  ///
  /// \param collapse_dims Fixed values for each axis (use \c NaN to preserve original values).
  /// \return A new view with collapsed points.
  std::shared_ptr<PointPerceptionsOpsView> collapse(
    const std::vector<double> & collapse_dims) const;

  /// \brief Retrieves all selected points across perceptions as a single concatenated cloud.
  /// \return Concatenated point cloud.
  pcl::PointCloud<pcl::PointXYZ> as_points() const;

  /// \brief Retrieves the filtered point cloud for a specific perception.
  /// \param idx Index of the target perception in the underlying container.
  /// \return Const reference to the filtered point cloud.
  const pcl::PointCloud<pcl::PointXYZ> & as_points(int idx) const;

  /// \brief Fuses all perceptions into one by transforming them to a common frame.
  /// \param target_frame Frame ID to which all clouds are transformed.
  /// \return New view containing the fused result.
  std::shared_ptr<PointPerceptionsOpsView> fuse(const std::string & target_frame) const;

  /// \brief Adds a new perception from a point cloud and returns an extended view.
  /// \param points Point cloud to include.
  /// \param frame Frame ID associated with \p points.
  /// \param stamp Timestamp associated with \p points.
  /// \return New view including the added perception.
  std::shared_ptr<PointPerceptionsOpsView> add(
    const pcl::PointCloud<pcl::PointXYZ> points,
    const std::string & frame,
    rclcpp::Time stamp) const;

  /// \brief Provides a constant reference to the underlying perceptions container.
  /// \return Constant reference to the container.
  const PointPerceptions & get_perceptions() const {return perceptions_;}

private:
  std::optional<PointPerceptions> owned_;      ///< Owned container if moved in.
  const PointPerceptions & perceptions_;       ///< Reference to perception container.
  std::vector<pcl::PointIndices> indices_;     ///< Filtered indices per perception.
};

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__POINTPERCEPTIONS_HPP_
