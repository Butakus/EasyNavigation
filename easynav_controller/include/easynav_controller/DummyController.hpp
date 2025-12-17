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
/// \brief Declaration of the DummyController method.

#ifndef EASYNAV_CONTROLLER__DUMMYCONTROLLER_HPP_
#define EASYNAV_CONTROLLER__DUMMYCONTROLLER_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_core/ControllerMethodBase.hpp"

namespace easynav
{

/**
 * @class DummyController
 * @brief A default "dummy" implementation for the Control Method.
 *
 * This control method does nothing. It serves as an example, and will be used as a default plugin implementation
 * if the navigation system configuration does not specify one.
 */
class DummyController : public easynav::ControllerMethodBase
{
public:
  DummyController() = default;
  ~DummyController() = default;

  /**
   * @brief Initializes the control method plugin.
   *
   * This method is called once during the configuration phase of the controller node,
   * and can be optionally overridden by derived classes to perform custom setup logic.
   */
  virtual void on_initialize() override;

  /**
   * @brief Run the control method and update the control command.
   *
   * This method will be called by the system's ControllerNode to run the control algorithm.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update_rt(NavState & nav_state) override;

private:
  /**
   * @brief Current robot velocity command.
   */
  geometry_msgs::msg::TwistStamped cmd_vel_;

  double cycle_time_rt_ {0.0};
  double cycle_time_nort_ {0.0};
};

}  // namespace easynav

#endif  // EASYNAV_CONTROLLER__DUMMYCONTROLLER_HPP_
