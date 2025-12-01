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


#include <string>

#include "cv_bridge/cv_bridge.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/DetectionsPerception.hpp"

namespace easynav
{


rclcpp::SubscriptionBase::SharedPtr
DetectionsPerceptionsHandler::create_subscription(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & topic,
  const std::string & type,
  std::shared_ptr<PerceptionBase> target,
  rclcpp::CallbackGroup::SharedPtr cb_group)
{
  if (type != "vision_msgs/msg/Detection3DArray") {
    throw std::runtime_error("Unsupported message type for DetectionsPerceptionsHandler: " + type);
  }

  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = cb_group;

  return node.create_subscription<vision_msgs::msg::Detection3DArray>(
    topic, rclcpp::QoS(1),
    [target](const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
      auto typed_target = std::dynamic_pointer_cast<DetectionsPerception>(target);

      typed_target->stamp = msg->header.stamp;
      typed_target->frame_id = msg->header.frame_id;
      typed_target->new_data = true;

      typed_target->data = *msg;  // Copy the Detection3DArray message
      typed_target->valid = true;
    }, options);
}

}  // namespace easynav
