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
