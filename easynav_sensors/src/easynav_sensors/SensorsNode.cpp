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
/// \brief Implementation of the SensorsNode class.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_sensors/SensorsNode.hpp"
#include "easynav_common/YTSession.hpp"

namespace easynav
{

using namespace std::chrono_literals;

SensorsNode::SensorsNode(const rclcpp::NodeOptions & options)
: LifecycleNode("sensors_node", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  percept_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "sensors_node/perceptions", rclcpp::SensorDataQoS().reliable());

  if (!has_parameter("sensors")) {
    declare_parameter("sensors", std::vector<std::string>());
  }

  if (!has_parameter("forget_time")) {
    declare_parameter("forget_time", 1.0);
  }

  if (!has_parameter("perception_default_frame")) {
    perception_default_frame_ = "odom";
    declare_parameter("perception_default_frame", perception_default_frame_);
  }

  perceptions_ = std::make_shared<Perceptions>();
}

SensorsNode::~SensorsNode()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
}


using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
CallbackReturnT
SensorsNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  std::vector<std::string> sensors;
  get_parameter("sensors", sensors);
  get_parameter("forget_time", forget_time_);
  get_parameter("perception_default_frame", perception_default_frame_);

  for (const auto & sensor : sensors) {
    std::string topic, msg_type;

    if (!has_parameter(sensor + ".topic")) {
      declare_parameter(sensor + ".topic", topic);
    }
    if (!has_parameter(sensor + ".type")) {
      declare_parameter(sensor + ".type", msg_type);
    }

    get_parameter(sensor + ".topic", topic);
    get_parameter(sensor + ".type", msg_type);

    auto atomic_perception = std::make_shared<std::atomic<std::shared_ptr<Perception>>>(
      std::make_shared<Perception>());

    PerceptionPtr perception_entry;
    perception_entry.perception = atomic_perception;

    if (msg_type == "LaserScan") {
      perception_entry.subscription = create_typed_subscription<sensor_msgs::msg::LaserScan>(
        *this, topic, atomic_perception, realtime_cbg_);
    } else if (msg_type == "PointCloud") {
      perception_entry.subscription = create_typed_subscription<sensor_msgs::msg::PointCloud2>(
        *this, topic, atomic_perception, realtime_cbg_);
    } else {
      RCLCPP_ERROR(get_logger(), "Sensor type [%s] not supported", msg_type.c_str());
      return CallbackReturnT::FAILURE;
    }

    perceptions_->push_back(perception_entry);
  }

  return CallbackReturnT::SUCCESS;
}


CallbackReturnT
SensorsNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  percept_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  percept_pub_->on_deactivate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
SensorsNode::get_real_time_cbg()
{
  return realtime_cbg_;
}

bool
SensorsNode::cycle_rt(std::shared_ptr<NavState> nav_state, bool trigger)
{
  EASYNAV_TRACE_EVENT;

  (void)trigger;

  bool trigger_perceptions = false;
  for (const auto & p : *perceptions_) {
    auto perception = p.perception->load();
    trigger_perceptions = trigger_perceptions || perception->new_data;
    perception->new_data = false;
  }

  nav_state->set_shared_ptr("perceptions", perceptions_);

  return trigger_perceptions;
}

void
SensorsNode::cycle(std::shared_ptr<NavState> nav_state)
{
  EASYNAV_TRACE_EVENT;

  for (auto & p : *perceptions_) {
    auto perception = p.perception->load();
    if (perception->valid && (now() - perception->stamp).seconds() > forget_time_) {
      perception->valid = false;
    }
  }

  nav_state->set_shared_ptr("perceptions", perceptions_);

  if (percept_pub_->get_subscription_count() > 0) {
    auto fused = PerceptionsOpsView(*perceptions_)
      .fuse(perception_default_frame_);

    auto fused_points = fused->as_points();

    auto msg = points_to_rosmsg(fused_points);
    msg.header.frame_id = perception_default_frame_;
    msg.header.stamp = fused->get_perceptions()[0].perception->load()->stamp;

    percept_pub_->publish(msg);
  }
}

}  // namespace easynav
