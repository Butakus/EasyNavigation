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


#include <string>

#include "cv_bridge/cv_bridge.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_sensors/types/DetectionsPerception.hpp"

namespace easynav
{


rclcpp::SubscriptionBase::SharedPtr
DetectionsPerceptionsHandler::create_subscription(
  const std::string & topic,
  const std::string & type,
  std::shared_ptr<PerceptionBase> target,
  rclcpp::CallbackGroup::SharedPtr cb_group)
{
  if (type != "vision_msgs/msg/Detection3DArray") {
    throw std::runtime_error("Unsupported message type for DetectionsPerceptionsHandler: " + type);
  }

  auto & node = *parent_node_;
  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = cb_group;

  const auto clock_type = node.get_clock()->get_clock_type();

  return node.create_subscription<vision_msgs::msg::Detection3DArray>(
    topic, rclcpp::QoS(1),
    [target, clock_type](const vision_msgs::msg::Detection3DArray::SharedPtr msg)
    {
      auto typed_target = std::dynamic_pointer_cast<DetectionsPerception>(target);

      typed_target->stamp = rclcpp::Time(msg->header.stamp, clock_type);
      typed_target->frame_id = msg->header.frame_id;
      typed_target->new_data = true;

      typed_target->data = *msg;  // Copy the Detection3DArray message
      typed_target->valid = true;
    }, options);
}

void
DetectionsPerceptionsHandler::populate_nav_state(
  const std::string & group,
  const std::vector<PerceptionPtr> & perceptions,
  NavState & ns)
{
  ns.set(group, get_perceptions<DetectionsPerception>(perceptions));
}

rclcpp::Time get_latest_detections_perceptions_stamp(const DetectionsPerceptions & perceptions)
{
  auto is_newer = [](const rclcpp::Time & a, const rclcpp::Time & b) {
      if (a.get_clock_type() == b.get_clock_type()) {
        return a > b;
      }
      return a.nanoseconds() > b.nanoseconds();
    };

  rclcpp::Time latest_stamp;
  bool inited = false;

  for (const auto & perception : perceptions) {
    if (!inited || is_newer(perception->stamp, latest_stamp)) {
      latest_stamp = perception->stamp;
      inited = true;
    }
  }
  return latest_stamp;
}

}  // namespace easynav

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(easynav::DetectionsPerceptionsHandler, easynav::PerceptionHandler)
