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
/// \brief Implementation of the SystemNode class.

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_system/SystemNode.hpp"

#include "easynav_controller/ControllerNode.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_maps_manager/MapsManagerNode.hpp"
#include "easynav_planner/PlannerNode.hpp"
#include "easynav_sensors/SensorsNode.hpp"
#include "easynav_common/YTSession.hpp"
#include "easynav_common/types/PointPerception.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


namespace easynav
{

using namespace std::chrono_literals;

SystemNode::SystemNode(const rclcpp::NodeOptions & options)
: LifecycleNode("system_node", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  nav_state_ = std::make_shared<NavState>();

  NavState::register_printer<PointPerceptions>(
    [](const PointPerceptions & perceptions) {
      std::ostringstream ret;
      ret << "PointPerception " << perceptions.size() << " with:\n";
      for (const auto & perception : perceptions) {
        ret << "\t[" << static_cast<const void *>(perception.get()) << "] --> "
            << perception->data.size() << " points in frame [" << perception->frame_id
            << "] with ts " << perception->stamp.seconds() << "\n";
      }
      return ret.str();
    });


  NavState::register_printer<nav_msgs::msg::Goals>(
    [](const nav_msgs::msg::Goals & goals) {
      std::string ret = "Goals " + std::to_string(goals.goals.size()) + " with :\n";
      for (const auto & goal : goals.goals) {
        std::string p_str = "\t--> (" + std::to_string(goal.pose.position.x) + ", " +
        std::to_string(goal.pose.position.y) + ")\n";
        ret = ret + p_str;
      }
      return ret;
    });

  controller_node_ = ControllerNode::make_shared();
  localizer_node_ = LocalizerNode::make_shared();
  maps_manager_node_ = MapsManagerNode::make_shared();
  planner_node_ = PlannerNode::make_shared();
  sensors_node_ = SensorsNode::make_shared();

  declare_parameter<std::string>("tf_prefix", "");
  declare_parameter<bool>("use_cmd_vel_stamped", use_cmd_vel_stamped_);

  // get_logger().set_level(rclcpp::Logger::Level::Debug);
}

SystemNode::~SystemNode()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
SystemNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  get_parameter<bool>("use_cmd_vel_stamped", use_cmd_vel_stamped_);

  std::string tf_prefix;
  get_parameter("tf_prefix", tf_prefix);
  if (tf_prefix != "") {
    tf_prefix = tf_prefix + "/";
  }

  for (auto & system_node : get_system_nodes()) {
    system_node.second.node_ptr->declare_parameter<std::string>("tf_prefix", "");
    system_node.second.node_ptr->set_parameter({"tf_prefix", tf_prefix});

    RCLCPP_INFO(get_logger(), "Configuring [%s]", system_node.first.c_str());
    system_node.second.node_ptr->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    if (system_node.second.node_ptr->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "Unable to configure [%s]", system_node.first.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  goal_manager_ = GoalManager::make_shared(*nav_state_, shared_from_this());

  navstate_pub_ = create_publisher<std_msgs::msg::String>(
    "easynav_navstate", 100);

  if (use_cmd_vel_stamped_) {
    vel_pub_stamped_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 100);
  } else {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100);
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  for (auto & system_node : get_system_nodes()) {
    RCLCPP_INFO(get_logger(), "Activating [%s]", system_node.first.c_str());
    system_node.second.node_ptr->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    if (system_node.second.node_ptr->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "Unable to activate [%s]", system_node.first.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  system_main_nort_timer_ = create_timer(30ms, std::bind(&SystemNode::system_cycle, this));

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  for (auto & system_node : get_system_nodes()) {
    RCLCPP_INFO(get_logger(), "Deactivating [%s]", system_node.first.c_str());
    system_node.second.node_ptr->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

    if (system_node.second.node_ptr->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "Unable to deactivate [%s]", system_node.first.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  system_main_nort_timer_->cancel();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
SystemNode::get_real_time_cbg()
{
  return realtime_cbg_;
}

void
SystemNode::system_cycle_rt()
{
  EASYNAV_TRACE_EVENT;

  RCLCPP_DEBUG(get_logger(), "SystemNode::system_cycle_rt\n%s", nav_state_->debug_string().c_str());

  bool trigger_perceptions = sensors_node_->cycle_rt(nav_state_);
  bool trigger_localization = localizer_node_->cycle_rt(nav_state_, trigger_perceptions);

  bool trigger_controller = false;
  bool robot_idle_stop = true;

  const auto navigation_state = nav_state_->get<GoalManager::State>("navigation_state");

  bool trigger = trigger_perceptions || trigger_localization;
  trigger_controller = controller_node_->cycle_rt(nav_state_, trigger);

  if (nav_state_->has("cmd_vel")) {
    geometry_msgs::msg::TwistStamped current_cmd_vel;
    current_cmd_vel = nav_state_->get<geometry_msgs::msg::TwistStamped>("cmd_vel");

    if (trigger_controller) {
      if (use_cmd_vel_stamped_ && vel_pub_stamped_->get_subscription_count()) {
        vel_pub_stamped_->publish(current_cmd_vel);
      }
      if (!use_cmd_vel_stamped_ && vel_pub_->get_subscription_count()) {
        vel_pub_->publish(current_cmd_vel.twist);
      }
    }
  }
}

void
SystemNode::system_cycle()
{
  EASYNAV_TRACE_EVENT;

  RCLCPP_DEBUG(get_logger(), "SystemNode::system_cycle\n%s", nav_state_->debug_string().c_str());

  sensors_node_->cycle(nav_state_);
  localizer_node_->cycle(nav_state_);
  maps_manager_node_->cycle(nav_state_);
  goal_manager_->update(*nav_state_);

  rclcpp::Time goals_ts(goal_manager_->get_goals().header.stamp);
  rclcpp::Time planner_ts = planner_node_->get_last_execution_ts();

  planner_node_->cycle(nav_state_, planner_ts < goals_ts);

  if (navstate_pub_->get_subscription_count() > 0) {
    std_msgs::msg::String msg;
    msg.data = nav_state_->debug_string();
    navstate_pub_->publish(msg);
  }
}

std::map<std::string, SystemNodeInfo>
SystemNode::get_system_nodes()
{
  std::map<std::string, SystemNodeInfo> ret;

  ret[controller_node_->get_name()] = {controller_node_, controller_node_->get_real_time_cbg()};
  ret[localizer_node_->get_name()] = {localizer_node_, localizer_node_->get_real_time_cbg()};
  ret[maps_manager_node_->get_name()] = {maps_manager_node_, nullptr};
  ret[planner_node_->get_name()] = {planner_node_, nullptr};
  ret[sensors_node_->get_name()] = {sensors_node_, sensors_node_->get_real_time_cbg()};

  return ret;
}

}  // namespace easynav
