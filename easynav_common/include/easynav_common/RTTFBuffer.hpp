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


#ifndef EASYNAV_COMMON_TYPES__RTTFBUFFER_HPP_
#define EASYNAV_COMMON_TYPES__RTTFBUFFER_HPP_

#include "easynav_common/Singleton.hpp"

#include "tf2_ros/buffer.hpp"

namespace easynav
{

/**
 * @class RTTFBuffer
 * @brief Provides functionality for RTTFBuffer.
 */
class RTTFBuffer : public tf2_ros::Buffer, public Singleton<RTTFBuffer>
{
public:
  explicit RTTFBuffer(const rclcpp::Clock::SharedPtr & clock)
  : tf2_ros::Buffer(clock)
  {}

  explicit RTTFBuffer()
  : Buffer(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME))
  {
    RCLCPP_WARN(rclcpp::get_logger("RTTFBuffer"),
      "You should be creating this RTTFBuffer with your clock."
      "Using default clock RCL_ROS_TIME");
  }

  SINGLETON_DEFINITIONS(RTTFBuffer)
};

}  // namespace easynav


#endif  // EASYNAV_COMMON_TYPES__RTTFBUFFER_HPP_
