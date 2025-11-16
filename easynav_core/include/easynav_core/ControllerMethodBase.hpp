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
/// \brief Declaration of the abstract base class ControllerMethodBase.

#ifndef EASYNAV_CORE__CONTROLLERMETHODBASE_HPP_
#define EASYNAV_CORE__CONTROLLERMETHODBASE_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"

namespace easynav
{

/**
 * @class ControllerMethodBase
 * @brief Abstract base class for control methods in Easy Navigation.
 *
 * This class defines the interface for control algorithm implementations.
 * Derived classes must implement the control logic and provide access to the computed command.
 */
class ControllerMethodBase : public MethodBase
{
public:
  /// @brief Default constructor.
  ControllerMethodBase() = default;

  /// @brief Virtual destructor.
  virtual ~ControllerMethodBase() = default;

  virtual std::expected<void, std::string>
  initialize(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
    const std::string & plugin_name,
    const std::string & tf_prefix = "");

  /**
   * @brief Helper to run the real-time control method if appropriate.
   *
   * Invokes update_rt() only if the method is due or forced by trigger.
   *
   * @param nav_state The current state of the navigation system.
   * @param trigger Force execution regardless of timing.
   * @return True if update_rt() was called, false otherwise.
   */
  bool internal_update_rt(NavState & nav_state, bool trigger = false);

protected:
  /**
   * @brief Run the control method and update the control command.
   *
   * Called by the system to compute a new control command using the current navigation state.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update_rt([[maybe_unused]] NavState & nav_state) {}


  double robot_radius_{0.3};
  double robot_height_{0.3};

  double brake_acc_{1.0};                   // m/s^2
  double safety_margin_{0.1};               // m
  double linear_speed_min_threshold_{0.05};  // m/s
  double angular_speed_min_threshold_{0.2};  // rad/s

  double angular_brake_acc_{1.0};           // rad/s^2 (si lo quieres usar)
  double z_min_filter_{0.0};                // m
  double rot_safety_margin_{0.05};          // m

  double downsample_leaf_size_{0.1};
  std::string motion_frame_{"base_link"};

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_marker_pub_;

  bool is_inminent_collision(NavState & nav_state);
  virtual void on_inminent_collision(NavState & nav_state);

void publish_collision_zone_marker(
  const geometry_msgs::msg::Pose & base_pose,
  double vx, double vy, double wz,
  double d_stop,
  bool imminent_collision);

};

}  // namespace easynav

#endif  // EASYNAV_CORE__CONTROLLERMETHODBASE_HPP_
