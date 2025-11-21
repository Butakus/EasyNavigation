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
/// \brief Defines data structures and utilities for representing and processing image perceptions.
///
/// This file contains the definition of the ImagePerception class, which holds image sensor data (as cv::Mat),
/// and the ImagePerceptionHandler class, which handles subscriptions to image messages and transforms them into
/// ImagePerception instances. It also defines an alias for a collection of such perceptions.

#ifndef EASYNAV_COMMON_TYPES__IMAGEPERCEPTIONS_HPP_
#define EASYNAV_COMMON_TYPES__IMAGEPERCEPTIONS_HPP_

#include <string>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

/// \class ImagePerception
/// \brief Represents a single image perception from a sensor.
///
/// Inherits from PerceptionBase and stores an OpenCV image (cv::Mat) along with metadata such as timestamp and frame_id.
class ImagePerception : public PerceptionBase
{
public:
  /// \brief Group identifier for image perceptions.
  static constexpr std::string_view default_group_ = "image";

  /// \brief Returns whether the given ROS 2 type name is supported by this perception.
  /// \param t Fully qualified message type name (e.g., "sensor_msgs/msg/Image").
  /// \return true if \p t equals "sensor_msgs/msg/Image", otherwise false.
  static inline bool supports_msg_type(std::string_view t)
  {
    return t == "sensor_msgs/msg/Image";
  }

  /// \brief Image data received from the sensor.
  ///
  /// The matrix layout follows OpenCV conventions. The encoding and channel depth depend on upstream conversion
  /// (typically via cv_bridge).
  cv::Mat data;
};

/// \class ImagePerceptionHandler
/// \brief Handles the creation and updating of ImagePerception instances from sensor_msgs::msg::Image messages.
///
/// This class provides methods to register subscriptions to image topics, decode incoming messages into cv::Mat
/// using cv_bridge, and update target ImagePerception instances.
class ImagePerceptionHandler : public PerceptionHandler
{
public:
  /// \brief Returns the group managed by this handler.
  /// \return The string literal "image".
  std::string group() const override {return "image";}

  /// \brief Creates a new empty ImagePerception instance.
  /// \param sensor_id Name or identifier of the sensor. Currently unused, reserved for future extensions.
  /// \return Shared pointer to a newly created ImagePerception.
  std::shared_ptr<PerceptionBase> create(const std::string &) override
  {
    return std::make_shared<ImagePerception>();
  }

  /// \brief Creates a subscription to an image topic that updates a target ImagePerception.
  ///
  /// The subscription receives sensor_msgs::msg::Image messages on \p topic, converts them to cv::Mat,
  /// fills ImagePerception::data, and updates inherited metadata (stamp, frame_id).
  ///
  /// \param node Lifecycle node used to create the subscription.
  /// \param topic Topic name to subscribe to.
  /// \param type ROS message type name. It must be "sensor_msgs/msg/Image".
  /// \param target Shared pointer to the ImagePerception to be updated.
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
 * @typedef ImagePerceptions
 * @brief Alias for a vector of shared pointers to ImagePerception objects.
 *
 * The container can represent a time-ordered or batched collection, depending on producer logic.
 */
using ImagePerceptions =
  std::vector<std::shared_ptr<ImagePerception>>;

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__IMAGEPERCEPTIONS_HPP_
