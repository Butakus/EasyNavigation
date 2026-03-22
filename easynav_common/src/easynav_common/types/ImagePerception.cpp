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
#include "sensor_msgs/msg/image.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/ImagePerception.hpp"

namespace easynav
{


rclcpp::SubscriptionBase::SharedPtr
ImagePerceptionHandler::create_subscription(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & topic,
  const std::string & type,
  std::shared_ptr<PerceptionBase> target,
  rclcpp::CallbackGroup::SharedPtr cb_group)
{
  if (type != "sensor_msgs/msg/Image") {
    throw std::runtime_error("Unsupported message type for ImagePerceptionHandler: " + type);
  }

  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = cb_group;

  const auto clock_type = node.get_clock()->get_clock_type();

  return node.create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::QoS(1),
    [target, clock_type](const sensor_msgs::msg::Image::SharedPtr msg)
    {
      auto typed_target = std::dynamic_pointer_cast<ImagePerception>(target);

      typed_target->stamp = rclcpp::Time(msg->header.stamp, clock_type);
      typed_target->frame_id = msg->header.frame_id;
      typed_target->new_data = true;

      try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        typed_target->data = cv_ptr->image.clone();  // se clona para evitar compartir buffers
        typed_target->valid = true;
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_WARN(
          rclcpp::get_logger("ImagePerceptionHandler"),
          "cv_bridge exception: %s", e.what());
        typed_target->valid = false;
      }
    },
    options);
}

rclcpp::Time get_latest_image_perceptions_stamp(const ImagePerceptions & perceptions)
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
