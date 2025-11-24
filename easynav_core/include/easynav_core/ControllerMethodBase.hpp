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

#include "visualization_msgs/msg/marker_array.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

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

  /**
   * @brief Initialize the controller method.
   *
   * Creates required publishers, reads configuration parameters and forwards
   * initialization to MethodBase.
   *
   * @param parent_node Reference to the parent lifecycle node.
   * @param plugin_name Plugin identifier used for namespacing parameters.
   * @param tf_prefix Optional TF prefix for frame resolution.
   * @return An empty value on success, or an error message otherwise.
   */
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

  /// @brief Enable or disable visualization markers for debugging.
  bool debug_markers_{false};

  /// @brief Enable or disable collision checking.
  bool collision_checker_active_{false};

  /// @brief Robot radius used for safety calculations (m).
  double robot_radius_{0.35};

  /// @brief Vertical extent of the robot used for filtering (m).
  double robot_height_{0.5};

  /// @brief Minimum Z considered when filtering point clouds (m).
  double z_min_filter_{0.0};

  /// @brief Maximum braking deceleration (m/s²).
  double brake_acc_{0.5};

  /// @brief Safety margin added to the braking distance (m).
  double safety_margin_{0.1};

  /// @brief Leaf size used to downsample point clouds (m).
  double downsample_leaf_size_{0.1};

  /// @brief Frame in which motion and collision checks are evaluated.
  std::string motion_frame_{"base_footprint"};

  /// @brief Publisher for collision visualization markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_marker_pub_;

  /**
   * @brief Detect whether an imminent collision is present.
   *
   * The check uses the robot velocity, braking distance, safety margins
   * and a filtered point cloud in the motion frame. It returns true if any
   * point lies within the collision prediction region.
   *
   * @param nav_state Current navigation state containing velocity and perceptions.
   * @return True if a collision is predicted, false otherwise.
   */
  bool is_inminent_collision(NavState & nav_state);

  /**
   * @brief Callback executed when a collision is detected.
   *
   * The default implementation stops the robot by setting a zero Twist.
   *
   * @param nav_state Reference to the navigation state to modify.
   */
  virtual void on_inminent_collision(NavState & nav_state);

  /**
   * @brief Publish visualization markers for the collision checking region.
   *
   * Publishes a CUBE representing the bounding box [min, max] used for
   * point cloud filtering, and a SPHERE_LIST containing the filtered points
   * used during the collision evaluation.
   *
   * @param min Lower bound of the collision check region [x, y, z].
   * @param max Upper bound of the collision check region [x, y, z].
   * @param cloud Filtered point cloud used during collision evaluation.
   * @param imminent_collision Whether the region is currently considered in collision.
   */
  void publish_collision_zone_marker(
    const std::vector<double> & min,
    const std::vector<double> & max,
    const pcl::PointCloud<pcl::PointXYZ> & cloud,
    bool imminent_collision);
};

}  // namespace easynav

#endif  // EASYNAV_CORE__CONTROLLERMETHODBASE_HPP_
