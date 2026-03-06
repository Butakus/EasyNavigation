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

#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "rclcpp/time.hpp"

#include "easynav_sensors/types/GNSSPerception.hpp"

namespace easynav
{


rclcpp::SubscriptionBase::SharedPtr
GNSSPerceptionHandler::create_subscription(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & topic,
  const std::string & type,
  std::shared_ptr<PerceptionBase> target,
  rclcpp::CallbackGroup::SharedPtr cb_group)
{
  if (type != "sensor_msgs/msg/NavSatFix") {
    throw std::runtime_error("Unsupported message type for GNSSPerceptionHandler: " + type);
  }

  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = cb_group;

  return node.create_subscription<sensor_msgs::msg::NavSatFix>(
    topic, rclcpp::QoS(1),
    [target](const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
      auto typed_target = std::dynamic_pointer_cast<GNSSPerception>(target);

      typed_target->stamp = msg->header.stamp;
      typed_target->frame_id = msg->header.frame_id;
      typed_target->new_data = true;
      typed_target->data = *msg;
      typed_target->valid = true;
    },
    options);
}

rclcpp::Time get_latest_gnss_perceptions_stamp(const GNSSPerceptions & perceptions)
{
  rclcpp::Time latest_stamp;
  bool inited = false;

  for (const auto & perception : perceptions) {
    if (!inited || perception->stamp > latest_stamp) {
      latest_stamp = perception->stamp;
      inited = true;
    }
  }

  return latest_stamp;
}

}  // namespace easynav
