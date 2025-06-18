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
/// \brief Declaration of the LocalizerNode class, a ROS 2 lifecycle node for localization tasks in Easy Navigation.

#ifndef EASYNAV_LOCALIZER__LOCALIZERNODE_HPP_
#define EASYNAV_LOCALIZER__LOCALIZERNODE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"

#include "easynav_core/LocalizerMethodBase.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/**
 * @class LocalizerNode
 * @brief ROS 2 lifecycle node that manages localization in Easy Navigation.
 *
 * Handles lifecycle transitions, dynamic loading of the localization plugin,
 * and periodic update execution in real-time or non-real-time contexts.
 */
class LocalizerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LocalizerNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor.
   * @param options Optional node configuration.
   */
  explicit LocalizerNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor.
  ~LocalizerNode();

  /**
   * @brief Configure the node.
   * @param state Lifecycle state.
   * @return SUCCESS if configuration succeeded.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activate the node.
   * @param state Lifecycle state.
   * @return SUCCESS if activation succeeded.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate the node.
   * @param state Lifecycle state.
   * @return SUCCESS if deactivation succeeded.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Clean up the node.
   * @param state Lifecycle state.
   * @return SUCCESS if cleanup succeeded.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shutdown the node.
   * @param state Lifecycle state.
   * @return SUCCESS if shutdown succeeded.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handle lifecycle error transition.
   * @param state Lifecycle state.
   * @return SUCCESS after handling error.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Get the callback group for real-time execution.
   * @return Shared pointer to the callback group.
   */
  rclcpp::CallbackGroup::SharedPtr get_real_time_cbg();

  /**
   * @brief Run a real-time localization cycle.
   * @param nav_state Shared pointer to the navigation state structure.
   * @param trigger Optional override to force execution.
   * @return True if plugin update was executed.
   */
  bool cycle_rt(std::shared_ptr<NavState> nav_state, bool trigger = false);

  /**
   * @brief Run a non-real-time localization cycle.
   * @param nav_state Shared pointer to the navigation state structure.
   */
  void cycle(std::shared_ptr<NavState> nav_state);

private:
  /// @brief Callback group reserved for real-time operations.
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /// @brief Instance of the loaded localization plugin.
  std::shared_ptr<LocalizerMethodBase> localizer_method_ {nullptr};

  /// @brief Plugin loader for LocalizerMethodBase implementations.
  std::unique_ptr<pluginlib::ClassLoader<easynav::LocalizerMethodBase>> localizer_loader_;
};

}  // namespace easynav

#endif  // EASYNAV_LOCALIZER__LOCALIZERNODE_HPP_
