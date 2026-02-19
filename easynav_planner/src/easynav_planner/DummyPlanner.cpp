// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// \brief Implementation of the DummyPlanner class.

#include <expected>
#include "easynav_planner/DummyPlanner.hpp"

namespace easynav
{

std::expected<void, std::string> DummyPlanner::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".cycle_time_rt", 0.0);
  node->declare_parameter<double>(plugin_name + ".cycle_time_nort", 0.0);
  node->get_parameter<double>(plugin_name + ".cycle_time_rt", cycle_time_rt_);
  node->get_parameter<double>(plugin_name + ".cycle_time_nort", cycle_time_nort_);

  // Initialize the Path message
  path_.header.stamp = get_node()->now();
  path_.header.frame_id = get_tf_prefix() + "map";
  path_.poses.clear();

  return {};
}

void DummyPlanner::update([[maybe_unused]] NavState & nav_state)
{
  auto start = get_node()->now();
  while ((get_node()->now() - start).seconds() < cycle_time_nort_) {}

  // Compute the current path...
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DummyPlanner, easynav::PlannerMethodBase)
