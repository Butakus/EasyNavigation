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
#include <vector>
#include <optional>

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
  std::shared_ptr<std::atomic<std::shared_ptr<PerceptionBase>>> target,
  rclcpp::CallbackGroup::SharedPtr cb_group)
{
  if (type != "sensor_msgs/msg/Image") {
    throw std::runtime_error("Unsupported message type for ImagePerceptionHandler: " + type);
  }

  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = cb_group;

  return node.create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::SensorDataQoS(),
    [target](const sensor_msgs::msg::Image::SharedPtr msg)
    {
      auto new_p = std::make_shared<ImagePerception>();
      new_p->stamp = msg->header.stamp;
      new_p->frame_id = msg->header.frame_id;
      new_p->new_data = true;

      try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        new_p->data = cv_ptr->image.clone();  // se clona para evitar compartir buffers
        new_p->valid = true;
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_WARN(
          rclcpp::get_logger("ImagePerceptionHandler"),
          "cv_bridge exception: %s", e.what());
        new_p->valid = false;
      }

      target->store(new_p);
    },
    options);
}

}  // namespace easynav
