// Copyright 2025 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// \brief Defines data structures and utilities for representing and processing sensor perceptions.
///
/// This file provides common interfaces for sensor perception handling:
/// - `PerceptionBase`: base class for sensor data.
/// - `PerceptionPtr`: utility for holding perception state and its subscription.
/// - `get_perceptions`: helper to extract typed collections from a heterogeneous container.
/// - `PerceptionHandler`: abstract base class for group-specific sensor handlers (pluginlib plugin).

#ifndef EASYNAV_SENSORS_TYPES__PERCEPTIONS_HPP_
#define EASYNAV_SENSORS_TYPES__PERCEPTIONS_HPP_

#include <string>

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "easynav_common/types/NavState.hpp"

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

/// \typedef PerceptionBasePtr
/// \brief Shared pointer alias to \ref PerceptionBase.
using PerceptionBasePtr = std::shared_ptr<PerceptionBase>;

/// \struct PerceptionPtr
/// \brief Represents a perception entry with its state and ROS subscription.
///
/// Holds a pointer to a perception object (\ref PerceptionBase) and the associated subscription.
/// Used internally by perception managers to update and access sensor data.
struct PerceptionPtr
{
  /// \brief Shared pointer to the current perception object.
  PerceptionBasePtr perception;

  /// \brief ROS 2 subscription to the sensor topic that provides data.
  rclcpp::SubscriptionBase::SharedPtr subscription;
};

/// \brief Extracts a homogeneous collection of perceptions of type \p T from a heterogeneous vector.
///
/// This helper iterates the input vector of \ref PerceptionPtr and:
/// - If \p T is exactly \ref PerceptionBase, returns all stored pointers without casting (heterogeneous view).
/// - Otherwise, attempts a `std::dynamic_pointer_cast<T>` and includes only those perceptions that match (homogeneous view).
///
/// \tparam T Target perception type. Must inherit from \ref PerceptionBase. Defaults to \ref PerceptionBase.
/// \param src Source vector containing heterogeneous perceptions and their subscriptions.
/// \return A vector of `std::shared_ptr<T>` containing the matching perceptions, in the same order as \p src.
template<typename T = PerceptionBase>
inline std::vector<std::shared_ptr<T>>
get_perceptions(const std::vector<PerceptionPtr> & src)
{
  static_assert(std::is_base_of_v<PerceptionBase, T>,
                "T must inherit from PerceptionBase");

  std::vector<std::shared_ptr<T>> out;
  out.reserve(src.size());

  for (const auto & h : src) {
    if (!h.perception) {continue;}

    if constexpr (std::is_same_v<T, PerceptionBase>) {
      // Heterogeneous: no cast needed
      out.push_back(h.perception);
    } else {
      // Homogeneous by derived type: include only successful casts
      if (auto p = std::dynamic_pointer_cast<T>(h.perception)) {
        out.push_back(std::move(p));
      }
    }
  }
  return out;
}

/// \class PerceptionHandler
/// \brief Abstract base class for pluginlib-based sensor perception handlers.
///
/// Each handler is responsible for a sensor group (e.g., "points", "image", "imu").
/// Concrete handlers are registered as pluginlib plugins and loaded at runtime.
/// A user can implement a new sensor type by deriving from this class and registering it
/// as a plugin in the corresponding package's plugin XML file.
class PerceptionHandler
{
public:
  virtual ~PerceptionHandler() = default;

  /// \brief Initializes the handler with the parent node and sensor name.
  ///
  /// Must be called once before any other method. Stores the node and sensor name,
  /// then delegates to \ref on_initialize for subclass-specific setup.
  ///
  /// \param parent_node Shared pointer to the lifecycle node managing this handler.
  /// \param sensor_name Name of the sensor (used as parameter namespace prefix).
  void initialize(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
    const std::string & sensor_name)
  {
    parent_node_ = parent_node;
    sensor_name_ = sensor_name;
    on_initialize();
  }

  /// \brief Optional post-initialization hook for subclasses.
  virtual void on_initialize() {}

  /// \brief Creates a new perception instance for this sensor.
  /// \return Shared pointer to a newly created \ref PerceptionBase subclass.
  virtual std::shared_ptr<PerceptionBase> create() = 0;

  /// \brief Creates a ROS subscription that stores incoming data into \p target.
  ///
  /// \param topic Topic name to subscribe to.
  /// \param type ROS message type name (e.g., `"sensor_msgs/msg/LaserScan"`).
  /// \param target Shared pointer to the \ref PerceptionBase subclass to update.
  /// \param cb_group Callback group for executor-level concurrency control.
  /// \return Shared pointer to the created subscription.
  virtual rclcpp::SubscriptionBase::SharedPtr create_subscription(
    const std::string & topic,
    const std::string & type,
    std::shared_ptr<PerceptionBase> target,
    rclcpp::CallbackGroup::SharedPtr cb_group) = 0;

  /// \brief Returns the group identifier associated with this handler.
  ///
  /// Example: `"points"`, `"image"`, `"imu"`, `"gnss"`.
  /// \return String representing the group name.
  virtual std::string group() const = 0;

  /// \brief Populates the \ref NavState with typed perceptions for the given group.
  ///
  /// Implementations call `ns.set(group, get_perceptions<ConcreteType>(perceptions))`
  /// to store the correctly typed vector into `NavState`.
  ///
  /// \param group The group name under which to store the perceptions.
  /// \param perceptions Vector of \ref PerceptionPtr for all sensors in this group.
  /// \param ns Navigation state to populate.
  virtual void populate_nav_state(
    const std::string & group,
    const std::vector<PerceptionPtr> & perceptions,
    NavState & ns) = 0;

  /// \brief Returns the sensor name provided during \ref initialize.
  const std::string & get_sensor_name() const {return sensor_name_;}

protected:
  /// \brief Returns the parent lifecycle node.
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() const {return parent_node_;}

  /// \brief Shared pointer to the parent lifecycle node.
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_{nullptr};

  /// \brief Name of the sensor (used as YAML parameter namespace prefix).
  std::string sensor_name_;
};

}  // namespace easynav

#endif  // EASYNAV_SENSORS_TYPES__PERCEPTIONS_HPP_
