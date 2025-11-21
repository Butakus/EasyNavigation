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
/// \brief Defines data structures and utilities for representing and processing IMU perceptions.
///
/// This file contains the definition of the IMUPerception class, which holds IMU sensor data,
/// and the IMUPerceptionHandler class, which handles subscriptions to IMU messages and transforms them into
/// IMUPerception instances. It also defines an alias for a collection of such perceptions.

#ifndef EASYNAV_COMMON_TYPES__IMUPERCEPTIONS_HPP_
#define EASYNAV_COMMON_TYPES__IMUPERCEPTIONS_HPP_

#include <string>
#include <vector>

#include "sensor_msgs/msg/imu.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

/// \class IMUPerception
/// \brief Represents a single IMU perception from a sensor.
///
/// Inherits from PerceptionBase and stores a sensor_msgs::msg::Imu message.
class IMUPerception : public PerceptionBase
{
public:
  /// \brief Group identifier for IMU perceptions.
  static constexpr std::string_view default_group_ = "imu";

  /// \brief Returns whether the given ROS 2 type name is supported by this perception.
  /// \param t Fully qualified message type name (e.g., "sensor_msgs/msg/Imu").
  /// \return true if \p t equals "sensor_msgs/msg/Imu", otherwise false.
  static inline bool supports_msg_type(std::string_view t)
  {
    return t == "sensor_msgs/msg/Imu";
  }

  /// \brief IMU data received from the sensor.
  sensor_msgs::msg::Imu data;
};

/// \class IMUPerceptionHandler
/// \brief Handles the creation and updating of IMUPerception instances from sensor_msgs::msg::Imu messages.
///
/// This class provides methods to register subscriptions to IMU topics and update IMUPerception objects.
class IMUPerceptionHandler : public PerceptionHandler
{
public:
  /// \brief Returns the group managed by this handler.
  /// \return The string literal "imu".
  std::string group() const override {return "imu";}

  /// \brief Creates a new empty IMUPerception instance.
  /// \param sensor_id Name or identifier of the sensor. Currently unused, reserved for future extensions.
  /// \return Shared pointer to a newly created IMUPerception.
  std::shared_ptr<PerceptionBase> create(const std::string &) override
  {
    return std::make_shared<IMUPerception>();
  }

  /// \brief Creates a subscription to an IMU topic that updates a target IMUPerception.
  ///
  /// The subscription receives sensor_msgs::msg::Imu messages on \p topic and writes the content into
  /// IMUPerception::data, updating inherited metadata (stamp, frame_id).
  ///
  /// \param node Lifecycle node used to create the subscription.
  /// \param topic Topic name to subscribe to.
  /// \param type ROS message type name. It must be "sensor_msgs/msg/Imu".
  /// \param target Shared pointer to the IMUPerception to be updated.
  /// \param cb_group Callback group for executor-level concurrency control.
  /// \return Shared pointer to the created subscription.
  rclcpp::SubscriptionBase::SharedPtr create_subscription(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & topic,
    const std::string & type,
    std::shared_ptr<PerceptionBase> target,
    rclcpp::CallbackGroup::SharedPtr cb_group) override;
};

/**
 * @typedef IMUPerceptions
 * @brief Alias for a vector of shared pointers to IMUPerception objects.
 *
 * The container can represent a time-ordered or batched collection, depending on producer logic.
 */
using IMUPerceptions =
  std::vector<std::shared_ptr<IMUPerception>>;

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__IMUPERCEPTIONS_HPP_
