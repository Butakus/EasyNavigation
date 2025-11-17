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

#include <string>
#include <vector>
#include <optional>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "pcl/common/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/PointPerception.hpp"
#include "easynav_common/YTSession.hpp"
#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

rclcpp::SubscriptionBase::SharedPtr
PointPerceptionHandler::create_subscription(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & topic,
  const std::string & type,
  std::shared_ptr<PerceptionBase> target,
  rclcpp::CallbackGroup::SharedPtr cb_group)
{
  auto options = rclcpp::SubscriptionOptions();
  options.callback_group = cb_group;

  if (type == "sensor_msgs/msg/PointCloud2") {
    return node.create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, rclcpp::SensorDataQoS().reliable(),
      [target](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
      {
        auto typed_target = std::dynamic_pointer_cast<PointPerception>(target);

        pcl::fromROSMsg(*msg, typed_target->data);
        typed_target->frame_id = msg->header.frame_id;
        typed_target->stamp = msg->header.stamp;
        typed_target->valid = true;
        typed_target->new_data = true;
      },
      options);
  }

  if (type == "sensor_msgs/msg/LaserScan") {
    return node.create_subscription<sensor_msgs::msg::LaserScan>(
      topic, rclcpp::SensorDataQoS().reliable(),
      [target](const sensor_msgs::msg::LaserScan::SharedPtr msg)
      {
        auto typed_target = std::dynamic_pointer_cast<PointPerception>(target);

        convert(*msg, typed_target->data);
        typed_target->frame_id = msg->header.frame_id;
        typed_target->stamp = msg->header.stamp;
        typed_target->valid = true;
        typed_target->new_data = true;
      },
      options);
  }

  throw std::runtime_error("Unsupported message type for PointPerceptionHandler [" + type + "]");
}

void
convert(const sensor_msgs::msg::LaserScan & scan, pcl::PointCloud<pcl::PointXYZ> & pc)
{
  const size_t num_points = scan.ranges.size();

  if (pc.points.size() != num_points) {
    pc.points.resize(num_points);
  }

  pc.header.frame_id = scan.header.frame_id;
  pc.width = static_cast<uint32_t>(pc.points.size());
  pc.height = 1;
  pc.is_dense = false;

  for (size_t i = 0; i < num_points; ++i) {
    float range = scan.ranges[i];

    pcl::PointXYZ & point = pc.points[i];

    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      point.x = std::numeric_limits<float>::quiet_NaN();
      point.y = std::numeric_limits<float>::quiet_NaN();
      point.z = std::numeric_limits<float>::quiet_NaN();
    } else {
      float angle = scan.angle_min + i * scan.angle_increment;
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);
      point.z = 0.0f;
    }
  }
}

sensor_msgs::msg::PointCloud2
perception_to_rosmsg(const PointPerception & perception)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(perception.data, msg);
  msg.header.frame_id = perception.frame_id;
  msg.header.stamp = perception.stamp;
  return msg;
}

sensor_msgs::msg::PointCloud2
points_to_rosmsg(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  return msg;
}


PointPerceptionsOpsView::PointPerceptionsOpsView(const PointPerceptions & perceptions)
: perceptions_(perceptions), indices_(perceptions.size())
{
  for (std::size_t i = 0; i < perceptions_.size(); ++i) {
    if (perceptions_[i]) {
      indices_[i].indices.resize(perceptions_[i]->data.size());
      std::iota(indices_[i].indices.begin(), indices_[i].indices.end(), 0);
    }
  }
}

PointPerceptionsOpsView::PointPerceptionsOpsView(const PointPerception & perception)
: owned_(std::in_place),
  perceptions_(*owned_),
  indices_(1)
{
  owned_->push_back(std::make_shared<PointPerception>(perception));

  indices_[0].indices.resize(perceptions_[0]->data.size());
  std::iota(indices_[0].indices.begin(), indices_[0].indices.end(), 0);
}

PointPerceptionsOpsView::PointPerceptionsOpsView(PointPerceptions && perceptions)
: owned_(std::move(perceptions)), perceptions_(*owned_), indices_(perceptions_.size())
{
  for (std::size_t i = 0; i < perceptions_.size(); ++i) {
    if (!perceptions_[i] || !perceptions_[i]->valid || perceptions_[i]->data.empty()) {continue;}

    indices_[i].indices.resize(perceptions_[i]->data.size());
    std::iota(indices_[i].indices.begin(), indices_[i].indices.end(), 0);
  }
}

PointPerceptionsOpsView &
PointPerceptionsOpsView::filter(
  const std::vector<double> & min_bounds,
  const std::vector<double> & max_bounds)
{
  for (std::size_t i = 0; i < perceptions_.size(); ++i) {
    if (!perceptions_[i] || !perceptions_[i]->valid || perceptions_[i]->data.empty()) {continue;}

    const auto & cloud = perceptions_[i]->data;
    auto & indices = indices_[i].indices;

    std::size_t write_idx = 0;
    for (std::size_t read_idx = 0; read_idx < indices.size(); ++read_idx) {
      const auto & pt = cloud[indices[read_idx]];
      bool keep = true;
      if (!std::isnan(min_bounds[0]) && pt.x < min_bounds[0]) {keep = false;}
      if (!std::isnan(max_bounds[0]) && pt.x > max_bounds[0]) {keep = false;}
      if (!std::isnan(min_bounds[1]) && pt.y < min_bounds[1]) {keep = false;}
      if (!std::isnan(max_bounds[1]) && pt.y > max_bounds[1]) {keep = false;}
      if (!std::isnan(min_bounds[2]) && pt.z < min_bounds[2]) {keep = false;}
      if (!std::isnan(max_bounds[2]) && pt.z > max_bounds[2]) {keep = false;}

      if (keep) {
        indices[write_idx++] = indices[read_idx];
      }
    }

    indices.resize(write_idx);
  }

  return *this;
}

PointPerceptionsOpsView &
PointPerceptionsOpsView::downsample(double resolution)
{
  for (std::size_t i = 0; i < perceptions_.size(); ++i) {
    if (!perceptions_[i] || !perceptions_[i]->valid || perceptions_[i]->data.empty()) {continue;}

    const auto & cloud = perceptions_[i]->data;
    auto & indices = indices_[i].indices;

    std::unordered_set<std::tuple<int, int, int>> voxel_set;
    std::size_t write_idx = 0;

    for (std::size_t read_idx = 0; read_idx < indices.size(); ++read_idx) {
      const auto & pt = cloud[indices[read_idx]];
      auto voxel = std::make_tuple(
        static_cast<int>(pt.x / resolution),
        static_cast<int>(pt.y / resolution),
        static_cast<int>(pt.z / resolution));

      if (voxel_set.insert(voxel).second) {
        indices[write_idx++] = indices[read_idx];
      }
    }

    indices.resize(write_idx);
  }

  return *this;
}

std::shared_ptr<PointPerceptionsOpsView>
PointPerceptionsOpsView::collapse(const std::vector<double> & collapse_dims) const
{
  PointPerceptions result;
  result.reserve(perceptions_.size());

  // Precompute collapse flags and values
  const bool collapse_x = collapse_dims.size() > 0 && !std::isnan(collapse_dims[0]);
  const bool collapse_y = collapse_dims.size() > 1 && !std::isnan(collapse_dims[1]);
  const bool collapse_z = collapse_dims.size() > 2 && !std::isnan(collapse_dims[2]);

  const float cx = collapse_x ? static_cast<float>(collapse_dims[0]) : 0.0f;
  const float cy = collapse_y ? static_cast<float>(collapse_dims[1]) : 0.0f;
  const float cz = collapse_z ? static_cast<float>(collapse_dims[2]) : 0.0f;

  for (std::size_t i = 0; i < perceptions_.size(); ++i) {
    const auto & pptr = perceptions_[i];
    if (!pptr || !pptr->valid || pptr->data.empty()) {
      continue;
    }

    const auto & idx_list = indices_[i].indices;
    if (idx_list.empty()) {
      continue;
    }

    auto collapsed = std::make_shared<PointPerception>();
    collapsed->valid = pptr->valid;
    collapsed->frame_id = pptr->frame_id;
    collapsed->stamp = pptr->stamp;

    const auto & cloud = pptr->data;

    // Reserve exactly the number of points that will be added
    collapsed->data.points.reserve(idx_list.size());
    collapsed->data.height = 1;
    collapsed->data.is_dense = cloud.is_dense;

    for (int idx : idx_list) {
      if (idx < 0 || static_cast<std::size_t>(idx) >= cloud.size()) {
        continue;
      }

      const auto & src = cloud[idx];
      pcl::PointXYZ dst = src;

      if (collapse_x) {dst.x = cx;}
      if (collapse_y) {dst.y = cy;}
      if (collapse_z) {dst.z = cz;}

      collapsed->data.push_back(dst);
    }

    collapsed->data.width = static_cast<uint32_t>(collapsed->data.points.size());

    if (!collapsed->data.empty()) {
      result.push_back(std::move(collapsed));
    }
  }

  return std::make_shared<PointPerceptionsOpsView>(std::move(result));
}

pcl::PointCloud<pcl::PointXYZ>
PointPerceptionsOpsView::as_points() const
{
  pcl::PointCloud<pcl::PointXYZ> output;

  for (std::size_t i = 0; i < perceptions_.size(); ++i) {
    auto perception = perceptions_[i];

    if (!perception || !perception->valid || perception->data.empty()) {continue;}

    const auto & cloud = perception->data;
    const auto & index_list = indices_[i].indices;

    for (int idx : index_list) {
      if (static_cast<std::size_t>(idx) < cloud.size()) {
        output.push_back(cloud[idx]);
      }
    }
  }

  return output;
}

std::shared_ptr<PointPerceptionsOpsView>
PointPerceptionsOpsView::fuse(const std::string & target_frame) const
{
  auto fused = std::make_shared<PointPerception>();
  fused->valid = true;
  fused->frame_id = target_frame;
  std::optional<rclcpp::Time> latest_stamp;

  // 1) Pre-compute total number of points after filters
  std::size_t total_points = 0;
  for (std::size_t i = 0; i < perceptions_.size(); ++i) {
    const auto & p = perceptions_[i];
    if (!p || !p->valid || p->data.empty()) {continue;}

    const auto & idx_list = indices_[i].indices;
    if (idx_list.empty()) {continue;}

    total_points += idx_list.size();
  }

  // Reserve memory once for the fused cloud
  fused->data.points.reserve(total_points);
  fused->data.height = 1;
  fused->data.is_dense = false;

  // 2) Transform / copy each perception into fused->data
  for (std::size_t i = 0; i < perceptions_.size(); ++i) {
    const auto & p = perceptions_[i];
    if (!p || !p->valid || p->data.empty()) {continue;}

    const auto & idx_list = indices_[i].indices;
    if (idx_list.empty()) {continue;}

    const auto & cloud = p->data;

    // Fast path: already in target_frame, just copy filtered points
    if (p->frame_id == target_frame) {
      for (int idx : idx_list) {
        if (static_cast<std::size_t>(idx) < cloud.size()) {
          fused->data.push_back(cloud[idx]);
        }
      }
    } else {
      // Need TF transform
      geometry_msgs::msg::TransformStamped tf_msg;
      try {
        tf_msg = RTTFBuffer::getInstance()->lookupTransform(
          target_frame, p->frame_id,
          tf2_ros::fromMsg(p->stamp),
          tf2::durationFromSec(0.0));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(
          rclcpp::get_logger("PointPerceptionsOpsView"),
          "TF failed: %s", ex.what());
        continue;
      }

      tf2::Transform tf;
      tf2::fromMsg(tf_msg.transform, tf);

      for (int idx : idx_list) {
        if (static_cast<std::size_t>(idx) >= cloud.size()) {
          continue;
        }
        const auto & pt = cloud[idx];
        tf2::Vector3 pt_tf(pt.x, pt.y, pt.z);
        tf2::Vector3 pt_out = tf * pt_tf;
        fused->data.emplace_back(
          static_cast<float>(pt_out.x()),
          static_cast<float>(pt_out.y()),
          static_cast<float>(pt_out.z()));
      }
    }

    if (!latest_stamp.has_value() || p->stamp > latest_stamp.value()) {
      latest_stamp = p->stamp;
    }
  }

  fused->data.width = static_cast<uint32_t>(fused->data.points.size());
  fused->stamp = latest_stamp.value_or(rclcpp::Time(0));

  PointPerceptions result;
  result.push_back(fused);

  return std::make_shared<PointPerceptionsOpsView>(std::move(result));
}

std::shared_ptr<PointPerceptionsOpsView>
PointPerceptionsOpsView::add(
  const pcl::PointCloud<pcl::PointXYZ> points,
  const std::string & frame,
  rclcpp::Time stamp) const
{
  auto new_perception = std::make_shared<PointPerception>();
  new_perception->valid = true;
  new_perception->frame_id = frame;
  new_perception->data = points;
  new_perception->stamp = stamp;

  PointPerceptions new_perceptions = perceptions_;
  new_perceptions.push_back(new_perception);
  auto new_perception_view = std::make_shared<PointPerceptionsOpsView>(std::move(new_perceptions));

  return new_perception_view;
}

PointPerceptions get_point_perceptions(std::vector<PerceptionPtr> & perceptionptr)
{
  return get_perceptions<PointPerception>(perceptionptr);
}

}  // namespace easynav
