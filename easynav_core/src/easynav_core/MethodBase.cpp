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
/// \brief Implementation of the base class MethodBase used in plugin-based EasyNav method components.

#include <memory>
#include <expected>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_core/MethodBase.hpp"

namespace easynav
{

std::expected<void, std::string>
MethodBase::initialize(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
  const std::string & plugin_name,
  const std::string & tf_prefix
)
{
  parent_node_ = parent_node;
  plugin_name_ = plugin_name;
  tf_prefix_ = tf_prefix;

  rt_frequency_ = 10.0;
  frequency_ = 10.0;

  parent_node_->declare_parameter(plugin_name + ".rt_freq", rt_frequency_);
  parent_node_->declare_parameter(plugin_name + ".freq", frequency_);
  parent_node_->get_parameter(plugin_name + ".rt_freq", rt_frequency_);
  parent_node_->get_parameter(plugin_name + ".freq", frequency_);

  last_ts_ = parent_node_->now();
  rt_last_ts_ = parent_node_->now();

  return on_initialize();
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
MethodBase::get_node() const
{
  return parent_node_;
}

const std::string &
MethodBase::get_plugin_name() const
{
  return plugin_name_;
}

const std::string &
MethodBase::get_tf_prefix() const
{
  return tf_prefix_;
}

bool
MethodBase::isTime2RunRT()
{
  if ((parent_node_->now() - rt_last_ts_).seconds() > (1.0 / rt_frequency_)) {
    rt_last_ts_ = parent_node_->now();
    return true;
  } else {
    return false;
  }
}

bool
MethodBase::isTime2Run()
{
  if ((parent_node_->now() - last_ts_).seconds() > (1.0 / frequency_)) {
    last_ts_ = parent_node_->now();
    return true;
  } else {
    return false;
  }
}

void
MethodBase::setRunRT()
{
  rt_last_ts_ = parent_node_->now();
}

void
MethodBase::setRun()
{
  last_ts_ = parent_node_->now();
}

}  // namespace easynav
