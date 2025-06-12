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

  NavState::register_printer<Perceptions>(
    [](const Perceptions & perceptions) {
      std::string ret = "Perception " + std::to_string(perceptions.size()) + " with :\n";
      for (const auto & perception : perceptions) {
        std::string p_str = "\t--> " + std::to_string(perception.perception->load()->data.size()) +
        " points [" + perception.perception->load()->frame_id + "] " +
        std::to_string(perception.perception->load()->stamp.seconds()) + "\n";
        ret = ret + p_str;
      }
      return ret;
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

  vel_pub_stamped_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 100);
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100);
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

  for (auto & system_node : get_system_nodes()) {
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

  bool trigger_perceptions = sensors_node_->cycle_rt(nav_state_);
  bool trigger_localization = localizer_node_->cycle_rt(nav_state_, trigger_perceptions);

  bool trigger_controller = false;
  bool robot_idle_stop = true;

  const auto & navigation_state = nav_state_->get<GoalManager::State>("navigation_state");

  // Hay que quitar esto
  if (navigation_state == GoalManager::State::IDLE) {
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = now();

    nav_state_->set("cmd_vel", cmd_vel);
  } else {
    bool trigger = trigger_perceptions || trigger_localization;
    trigger_controller = controller_node_->cycle_rt(nav_state_, trigger);
  }

  if (trigger_controller || !robot_idle_stop) {
    if (vel_pub_stamped_->get_subscription_count()) {
      vel_pub_stamped_->publish(nav_state_->get_ref<geometry_msgs::msg::TwistStamped>("cmd_vel"));
    }
    if (vel_pub_->get_subscription_count()) {
      vel_pub_->publish(nav_state_->get_ref<geometry_msgs::msg::TwistStamped>("cmd_vel").twist);
    }
  }
}

void
SystemNode::system_cycle()
{
  EASYNAV_TRACE_EVENT;

  sensors_node_->cycle(nav_state_);
  localizer_node_->cycle(nav_state_);
  maps_manager_node_->cycle(nav_state_);

  const auto & navigation_state = nav_state_->get_ref<GoalManager::State>("navigation_state");

  goal_manager_->update(*nav_state_);
  planner_node_->cycle(nav_state_);
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
