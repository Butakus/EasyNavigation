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

#ifndef EASYNAV_COMMON__TYPES__TFINFO_HPP_
#define EASYNAV_COMMON__TYPES__TFINFO_HPP_

#include <string>

namespace easynav
{

/// @brief Aggregated TF configuration used across EasyNav.
struct TFInfo
{
  // These parameters are designed to enforce compliance with REP-105:
  // http://www.ros.org/reps/rep-0105.html

  /// Optional TF prefix applied to frame names.
  std::string tf_prefix {""};

  /// Global map frame.
  std::string map_frame {"map"};

  /// Odometry frame.
  std::string odom_frame {"odom"};

  /// Robot base frame (base_link/base_footprint equivalent).
  std::string robot_frame {"base_link"};

  /// World frame used by global estimators (e.g. earth-fixed frame).
  std::string world_frame{"earth"};
};

}  // namespace easynav

#endif  // EASYNAV_COMMON__TYPES__TFINFO_HPP_
