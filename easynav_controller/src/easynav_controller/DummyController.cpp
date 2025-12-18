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
/// \brief Implementation of the DummyController class.

#include "easynav_controller/DummyController.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

void DummyController::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".cycle_time_rt", 0.0);
  node->get_parameter<double>(plugin_name + ".cycle_time_rt", cycle_time_rt_);
}

void DummyController::update_rt([[maybe_unused]] NavState & nav_state)
{
  namespace chr = std::chrono;
  auto start = chr::steady_clock::now();

  const auto & tf_info = easynav::RTTFBuffer::getInstance()->get_tf_info();

  // Compute the current command...
  cmd_vel_.header.stamp = get_node()->now();
  cmd_vel_.header.frame_id = tf_info.robot_frame;
  cmd_vel_.twist.linear.x = 0.0;
  cmd_vel_.twist.linear.y = 0.0;
  cmd_vel_.twist.linear.z = 0.0;
  cmd_vel_.twist.angular.x = 0.0;
  cmd_vel_.twist.angular.y = 0.0;
  cmd_vel_.twist.angular.z = 0.0;

  // cmd_vel_.angular.z = 1.0;

  // nav_state.set("cmd_vel", cmd_vel_);

  // Busy wait to simulate processing time
  while (chr::duration<double>(chr::steady_clock::now() - start).count() < cycle_time_rt_) {}
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DummyController, easynav::ControllerMethodBase)
