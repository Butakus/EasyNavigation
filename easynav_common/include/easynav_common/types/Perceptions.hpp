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
///
/// This file provides common interfaces for sensor perception handling:
/// - `PerceptionBase`: base class for sensor data.
/// - `PerceptionPtr`: utility for holding perception state and its subscription.
/// - `PerceptionHandler`: abstract base class for group-specific sensor handlers.

#ifndef EASYNAV_COMMON_TYPES__PERCEPTIONS_HPP_
#define EASYNAV_COMMON_TYPES__PERCEPTIONS_HPP_

#include <string>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav
{

/// \class PerceptionBase
/// \brief Abstract base class for representing a single sensor perception.
///
/// Contains common metadata (timestamp, frame ID, validity flags) that all perception types share.
class PerceptionBase
{
public:
  virtual ~PerceptionBase() = default;

  /// \brief Timestamp of the perception (ROS time).
  rclcpp::Time stamp;

  /// \brief Coordinate frame associated with the perception.
  std::string frame_id;

  /// \brief Whether the perception contains valid data.
  bool valid = false;

  /// \brief Whether the data has changed since the last observation.
  bool new_data = false;
};

/// \struct PerceptionPtr
/// \brief Represents a perception entry with its state and ROS subscription.
///
/// Holds an pointer to a perception object (`PerceptionBase`) and the associated subscription.
/// Used internally by perception managers to update and access sensor data.
struct PerceptionPtr
{
  /// \brief Atomic shared pointer to the current perception object.
  std::shared_ptr<PerceptionBase> perception;

  /// \brief ROS 2 subscription to the sensor topic that provides data.
  rclcpp::SubscriptionBase::SharedPtr subscription;
};

/// \class PerceptionHandler
/// \brief Abstract base interface for group-specific perception handlers.
///
/// Each handler is responsible for a sensor group (e.g., "points", "image", "dummy").
/// It creates appropriate `PerceptionBase` instances and handles the conversion from ROS messages.
class PerceptionHandler
{
public:
  virtual ~PerceptionHandler() = default;

  /// \brief Creates a new instance of a perception object managed by this handler.
  /// \param sensor_id Name or ID of the sensor (optional use by handler).
  /// \return Shared pointer to a new instance of a class derived from PerceptionBase.
  virtual std::shared_ptr<PerceptionBase> create(const std::string & sensor_id) = 0;

  /// \brief Creates a subscription that processes messages into PerceptionBase instances.
  ///
  /// The handler is expected to parse the message received on `topic` of type `type`
  /// and store the result in the `target`.
  ///
  /// \param node Reference to the lifecycle node used for creating the subscription.
  /// \param topic Topic name to subscribe to.
  /// \param type Type name of the ROS message (e.g., "sensor_msgs/msg/LaserScan").
  /// \param target Atomic shared pointer where perception results are stored.
  /// \param cb_group Callback group where the subscription callback will be executed.
  /// \return The created subscription.
  virtual rclcpp::SubscriptionBase::SharedPtr create_subscription(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & topic,
    const std::string & type,
    std::shared_ptr<PerceptionBase> target,
    rclcpp::CallbackGroup::SharedPtr cb_group) = 0;

  /// \brief Returns the group identifier associated with this handler.
  ///
  /// This identifier is used to match sensors to the appropriate handler.
  /// Example: "points", "image", "dummy".
  ///
  /// \return String representing the group name.
  virtual std::string group() const = 0;
};

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__PERCEPTIONS_HPP_
