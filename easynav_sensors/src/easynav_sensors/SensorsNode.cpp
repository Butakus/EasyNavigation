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

#include <tuple>
#include <string_view>
#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_sensors/SensorsNode.hpp"
#include "easynav_common/YTSession.hpp"

#include "easynav_common/types/ImagePerception.hpp"
#include "easynav_common/types/PointPerception.hpp"
#include "easynav_common/types/IMUPerception.hpp"

namespace easynav
{

using Registry = std::tuple<
  easynav::ImagePerception,
  easynav::IMUPerception,
  easynav::PointPerception
>;

namespace
{
static std::unordered_map<std::string, std::string> g_group_alias;
}

inline std::string
resolve_group_from_msg([[maybe_unused]] std::string_view msg_type, std::true_type)
{
  return {};
}

template<std::size_t I = 0>
std::string resolve_group_from_msg(std::string_view msg_type)
{
  if constexpr (I == std::tuple_size_v<Registry>) {
    return {};
  } else {
    using P = std::tuple_element_t<I, Registry>;
    if (P::supports_msg_type(msg_type)) {
      return std::string(P::default_group_);
    }
    return resolve_group_from_msg<I + 1>(msg_type);
  }
}

template<std::size_t I = 0>
bool set_by_group(
  const std::string & group,
  const std::vector<easynav::PerceptionPtr> & src,
  ::easynav::NavState & ns)
{
  if constexpr (I >= std::tuple_size_v<Registry>) {
    return false;
  } else {
    using P = std::tuple_element_t<I, Registry>;

    const bool match_direct = (group == P::default_group_);
    const bool match_alias =
      (!match_direct) &&
      (g_group_alias.find(group) != g_group_alias.end()) &&
      (g_group_alias[group] == P::default_group_);

    if (match_direct || match_alias) {
      ns.set(group, get_perceptions<P>(src));
      return true;
    }
    return set_by_group<I + 1>(group, src, ns);
  }
}

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

  ::easynav::NavState::register_printer<easynav::PointPerceptions>(
    [](const easynav::PointPerceptions & perceptions) {
      std::ostringstream ret;
      ret << "PointPerception " << perceptions.size() << " with:\n";
      for (const auto & perception : perceptions) {
        ret   << "\t[" << static_cast<const void *>(perception.get()) << "] --> "
              << perception->data.size() << " points in frame [" << perception->frame_id
              << "] with ts " << perception->stamp.seconds() << "\n";
      }
      return ret.str();
      });

  ::easynav::NavState::register_printer<easynav::ImagePerceptions>(
    [](const easynav::ImagePerceptions & perceptions) {
      std::ostringstream ret;
      ret << "ImagePerceptions " << perceptions.size() << " with:\n";
      for (const auto & perception : perceptions) {
        ret   << "\t[" << static_cast<const void *>(perception.get()) << "] --> "
              << "Image (" << perception->data.cols << " x  " << perception->data.rows << ")"
              << "] with ts " << perception->stamp.seconds() << "\n";
      }
      return ret.str();
      });

  ::easynav::NavState::register_printer<easynav::IMUPerceptions>(
    [](const easynav::IMUPerceptions & perceptions) {
      std::ostringstream ret;
      ret << "IMUPerceptions " << perceptions.size() << " with:\n";
      for (const auto & perception : perceptions) {
        ret   << "\t[" << static_cast<const void *>(perception.get()) << "] --> "
              << "IMUPerception linear acc = (" <<
          perception->data.linear_acceleration.x << ", " <<
          perception->data.linear_acceleration.y << ", " <<
          perception->data.linear_acceleration.z << ")\n";
      }
      return ret.str();
      });


  register_handler(std::make_shared<PointPerceptionHandler>());
  register_handler(std::make_shared<ImagePerceptionHandler>());
  register_handler(std::make_shared<IMUPerceptionHandler>());
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

  g_group_alias.clear();

  std::vector<std::string> sensors;
  get_parameter("sensors", sensors);
  get_parameter("forget_time", forget_time_);
  get_parameter("perception_default_frame", perception_default_frame_);

  get_parameter("tf_prefix", tf_prefix_);

  for (const auto & sensor_id : sensors) {
    std::string topic, msg_type, group;

    if (!has_parameter(sensor_id + ".topic")) {
      declare_parameter(sensor_id + ".topic", topic);
    }
    if (!has_parameter(sensor_id + ".type")) {
      declare_parameter(sensor_id + ".type", msg_type);
    }

    get_parameter(sensor_id + ".topic", topic);
    get_parameter(sensor_id + ".type", msg_type);

    group = resolve_group_from_msg(msg_type);

    if (!has_parameter(sensor_id + ".group")) {
      declare_parameter(sensor_id + ".group", group);
    }

    get_parameter(sensor_id + ".group", group);

    auto handler_it = handlers_.find(group);
    if (handler_it == handlers_.end()) {
      const std::string canonical = resolve_group_from_msg(msg_type);
      if (!canonical.empty()) {
        auto hit2 = handlers_.find(canonical);
        if (hit2 != handlers_.end()) {
          handlers_[group] = hit2->second;
          g_group_alias[group] = canonical;
          handler_it = handlers_.find(group);
          RCLCPP_INFO(get_logger(),
                      "Aliased group '%s' -> '%s' for type '%s'",
                      group.c_str(), canonical.c_str(), msg_type.c_str());
        }
      }
    }

    if (handler_it == handlers_.end()) {
      RCLCPP_WARN(get_logger(), "No handler for group [%s]", group.c_str());
      continue;
    }

    auto ptr = handler_it->second->create(sensor_id);
    auto sub = handler_it->second->create_subscription(*this, topic, msg_type, ptr,
      realtime_cbg_);

    perceptions_[group].emplace_back(PerceptionPtr{ptr, sub});
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
  (void)trigger;

  bool trigger_perceptions = false;

  for (auto & group_perceptions : perceptions_) {
    for (auto & p : group_perceptions.second) {
      trigger_perceptions = trigger_perceptions || p.perception->new_data;
      p.perception->new_data = false;
    }

    if (!set_by_group(group_perceptions.first, group_perceptions.second, *nav_state)) {
      RCLCPP_WARN(get_logger(), "No perception handler for group [%s]",
        group_perceptions.first.c_str());
    }
  }

  return trigger_perceptions;
}

void
SensorsNode::cycle(std::shared_ptr<NavState> nav_state)
{
  for (auto & group_perceptions : perceptions_) {
    for (auto & p : group_perceptions.second) {
      if (p.perception->valid && (now() - p.perception->stamp).seconds() > forget_time_) {
        p.perception->valid = false;
      }
    }
    if (!set_by_group(group_perceptions.first, group_perceptions.second, *nav_state)) {
      RCLCPP_WARN(get_logger(), "No perception handler for group [%s]",
              group_perceptions.first.c_str());
    }
  }

  if (percept_pub_->get_subscription_count() > 0) {
    auto fused = PointPerceptionsOpsView(get_point_perceptions(perceptions_["points"]))
      .fuse(tf_prefix_ + perception_default_frame_);

    auto fused_points = fused->as_points();

    auto msg = points_to_rosmsg(fused_points);
    msg.header.frame_id = tf_prefix_ + perception_default_frame_;
    msg.header.stamp = fused->get_perceptions()[0]->stamp;

    percept_pub_->publish(msg);
  }
}

void
SensorsNode::register_handler(std::shared_ptr<PerceptionHandler> handler)
{
  handlers_[handler->group()] = handler;
}

}  // namespace easynav
