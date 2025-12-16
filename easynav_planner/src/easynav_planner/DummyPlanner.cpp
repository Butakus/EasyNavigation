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
/// \brief Implementation of the DummyPlanner class.

#include "easynav_planner/DummyPlanner.hpp"
#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

void DummyPlanner::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".cycle_time_nort", 0.0);
  node->get_parameter<double>(plugin_name + ".cycle_time_nort", cycle_time_nort_);

  // Initialize the Path message
  path_.header.stamp = get_node()->now();
  path_.header.frame_id = easynav::RTTFBuffer::getInstance()->get_tf_info().map_frame;
  path_.poses.clear();
}

void DummyPlanner::update([[maybe_unused]] NavState & nav_state)
{
  namespace chr = std::chrono;
  auto start = chr::steady_clock::now();

  // Compute the current path...

  // Busy wait to simulate processing time
  while (chr::duration<double>(chr::steady_clock::now() - start).count() < cycle_time_nort_) {}
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DummyPlanner, easynav::PlannerMethodBase)
