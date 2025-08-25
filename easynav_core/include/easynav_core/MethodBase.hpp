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
/// \brief Declaration of the base class MethodBase used in plugin-based EasyNav method components.

#ifndef EASYNAV_CORE__METHODBASE_HPP_
#define EASYNAV_CORE__METHODBASE_HPP_

#include <memory>
#include <expected>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav
{

/**
 * @class MethodBase
 * @brief Base class for Easy Navigation method plugins.
 *
 * Provides lifecycle integration and update-rate management
 * for components such as localizers, controllers, or map managers.
 */
class MethodBase
{
public:
  /// @brief Default constructor.
  MethodBase() = default;

  /// @brief Virtual destructor.
  virtual ~MethodBase() = default;

  /**
   * @brief Initializes the method with the given node and plugin name.
   *
   * Also reads update frequencies and sets up internal state.
   *
   * @param parent_node Shared pointer to the parent lifecycle node.
   * @param plugin_name Name of the plugin (used for parameters and logging).
   * @return std::expected<void, std::string> indicating success or failure.
   */
  virtual std::expected<void, std::string>
  initialize(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
    const std::string & plugin_name,
    const std::string & tf_prefix = "");

  /**
   * @brief Hook for custom setup logic in derived classes.
   *
   * Called from initialize(). Can be overridden to implement extra initialization steps.
   *
   * @return std::expected<void, std::string> indicating success or failure.
   */
  virtual std::expected<void, std::string> on_initialize() {return {};}

  /**
   * @brief Get a shared pointer to the parent lifecycle node.
   *
   * @return Shared pointer to the lifecycle node.
   */
  [[nodiscard]] std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
  get_node() const;

  /**
   * @brief Get the name assigned to the plugin.
   *
   * @return Plugin name as a constant reference.
   */
  [[nodiscard]] const std::string &
  get_plugin_name() const;

  /**
   * @brief Get the TF namespace.
   *
   * @return TF namespace with a trailing "/".
   */
  [[nodiscard]] const std::string &
  get_tf_prefix() const;

  /**
   * @brief Check whether it is time to run a real-time update.
   *
   * Uses the configured real-time frequency to determine whether sufficient time has elapsed.
   *
   * @return True if update should run, false otherwise.
   */
  bool isTime2RunRT();

  /**
   * @brief Check whether it is time to run a normal (non-RT) update.
   *
   * Uses the configured frequency to determine whether sufficient time has elapsed.
   *
   * @return True if update should run, false otherwise.
   */
  bool isTime2Run();

  /**
   * @brief Mark that the real-time update has just run.
   *
   * Records the current time as the last RT execution timestamp. Call this right after
   * completing a successful RT iteration.
   *
   * @post @ref get_last_rt_execution_ts() reflects the time of this call.
   */
  void setRunRT();

  /**
   * @brief Mark that the normal (non-RT) update has just run.
   *
   * Records the current time as the last non-RT execution timestamp. Call this right after
   * completing a successful non-RT iteration.
   *
   * @post @ref get_last_execution_ts() reflects the time of this call.
   */
  void setRun();

  /**
   * @brief Get the timestamp of the last real-time execution.
   * @return Reference to the last RT execution time as recorded by @ref setRunRT().
   * @note If no RT run has been recorded yet, this value may be zero-initialized.
   */
  const rclcpp::Time & get_last_rt_execution_ts() const {return rt_last_ts_;}

  /**
   * @brief Get the timestamp of the last non-RT execution.
   * @return Reference to the last non-RT execution time as recorded by @ref setRun().
   * @note If no non-RT run has been recorded yet, this value may be zero-initialized.
   */
  const rclcpp::Time & get_last_execution_ts() const {return last_ts_;}

private:
  /// @brief Shared pointer to the parent lifecycle node.
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_ {nullptr};

  /// @brief Name assigned to the plugin.
  std::string plugin_name_;

  /// @brief TF Namespace.
  std::string tf_prefix_;

  /// @brief Desired real-time and non-RT loop frequencies in Hz.
  float rt_frequency_, frequency_;

  /// @brief Timestamps of the last executions.
  rclcpp::Time rt_last_ts_, last_ts_;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__METHODBASE_HPP_
