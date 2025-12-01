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
/// \brief Defines data structures and utilities for representing and processing GNSS perceptions.
///
/// This file contains the definition of the GNSSPerception class, which holds GNSS sensor data,
/// and the GNSSPerceptionHandler class, which handles subscriptions to GNSS messages and transforms them into
/// GNSSPerception instances. It also defines an alias for a collection of such perceptions.

#ifndef EASYNAV_COMMON_TYPES__GNSSPERCEPTIONS_HPP_
#define EASYNAV_COMMON_TYPES__GNSSPERCEPTIONS_HPP_

#include <string>
#include <vector>

#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

/// \class GNSSPerception
/// \brief Represents a single GNSS perception from a sensor.
///
/// Inherits from PerceptionBase and stores a sensor_msgs/msg/NavSatFix message.
class GNSSPerception : public PerceptionBase
{
public:
  /// \brief Group identifier for GNSS perceptions.
  static constexpr std::string_view default_group_ = "gnss";

  /// \brief Returns whether the given ROS 2 type name is supported by this perception.
  /// \param t Fully qualified message type name (e.g., "sensor_msgs/msg/NavSatFix").
  /// \return true if \p t equals "sensor_msgs/msg/NavSatFix", otherwise false.
  static inline bool supports_msg_type(std::string_view t)
  {
    return t == "sensor_msgs/msg/NavSatFix";
  }

  /// \brief GNSS data received from the sensor.
  sensor_msgs::msg::NavSatFix data;
};

/// \class GNSSPerceptionHandler
/// \brief Handles the creation and updating of GNSSPerception instances from sensor_msgs::msg::NavSatFix messages.
///
/// This class provides methods to register subscriptions to GNSS topics and update GNSSPerception objects.
class GNSSPerceptionHandler : public PerceptionHandler
{
public:
  /// \brief Returns the group managed by this handler.
  /// \return The string literal "gnss".
  std::string group() const override {return "gnss";}

  /// \brief Creates a new empty GNSSPerception instance.
  /// \param sensor_id Name or identifier of the sensor. Currently unused, reserved for future extensions.
  /// \return Shared pointer to a newly created GNSSPerception.
  std::shared_ptr<PerceptionBase> create(const std::string &) override
  {
    return std::make_shared<GNSSPerception>();
  }

  /// \brief Creates a subscription to an GNSS topic that updates a target GNSSPerception.
  ///
  /// The subscription receives sensor_msgs::msg::NavSatFix messages on \p topic and writes the content into
  /// GNSSPerception::data, updating inherited metadata (stamp, frame_id).
  ///
  /// \param node Lifecycle node used to create the subscription.
  /// \param topic Topic name to subscribe to.
  /// \param type ROS message type name. It must be "sensor_msgs/msg/NavSatFix".
  /// \param target Shared pointer to the GNSSPerception to be updated.
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
 * @typedef GNSSPerceptions
 * @brief Alias for a vector of shared pointers to GNSSPerception objects.
 *
 * The container can represent a time-ordered or batched collection, depending on producer logic.
 */
using GNSSPerceptions =
  std::vector<std::shared_ptr<GNSSPerception>>;

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__GNSSPERCEPTIONS_HPP_
