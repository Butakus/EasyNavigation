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
/// \brief Implementation of the abstract base class ControllerMethodBase.

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_common/YTSession.hpp"

#include "easynav_core/MethodBase.hpp"
#include "easynav_core/ControllerMethodBase.hpp"

#include "easynav_common/types/PointPerception.hpp"

namespace easynav
{

std::expected<void, std::string>
ControllerMethodBase::initialize(
  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
  const std::string & plugin_name,
  const std::string & tf_prefix)
{
  collision_marker_pub_ = parent_node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "collision_area", 10);




  return MethodBase::initialize(parent_node, plugin_name, tf_prefix);
}

bool
ControllerMethodBase::internal_update_rt(NavState & nav_state, bool trigger)
{
  if (isTime2RunRT() || trigger) {
    EASYNAV_TRACE_EVENT;

    update_rt(nav_state);

    if (is_inminent_collision(nav_state)) {
       on_inminent_collision(nav_state);
    }

    return true;
  } else {
    return false;
  }
}

void
ControllerMethodBase::on_inminent_collision(NavState & nav_state)
{

  RCLCPP_WARN_THROTTLE(
    get_node()->get_logger(), *get_node()->get_clock(), 1000,
    "ControllerMethodBase::on_inminent_collision: Inminent collision!! Stopping");
  nav_state.set("cmd_vel", geometry_msgs::msg::TwistStamped());
}

bool
ControllerMethodBase::is_inminent_collision(NavState & nav_state)
{
  std::cerr << "[COLLISION] ========================================" << std::endl;

  bool imminent = false;

  if (!nav_state.has("cmd_vel")) {
    std::cerr << "[COLLISION] No cmd_vel in NavState\n";
    return false;
  }
  if (!nav_state.has("points")) {
    std::cerr << "[COLLISION] No point perceptions in NavState\n";
    return false;
  }

  const auto twist = nav_state.get<geometry_msgs::msg::TwistStamped>("cmd_vel");
  const auto & perceptions = nav_state.get<PointPerceptions>("points");

  if (perceptions.empty()) {
    std::cerr << "[COLLISION] Perceptions empty\n";
    return false;
  }

  const double vx = twist.twist.linear.x;
  const double vy = twist.twist.linear.y;
  const double wz = twist.twist.angular.z;
  const double v_norm = std::sqrt(vx * vx + vy * vy);

  // Early-out solo si el robot está prácticamente estático (sin avanzar ni girar)
  if (v_norm < linear_speed_min_threshold_ &&
      std::fabs(wz) < angular_speed_min_threshold_) {
    std::cerr << "[COLLISION] Robot almost static (v≈0, w≈0) → no collision\n";
    return false;
  }

  const double a_brake = std::max(brake_acc_, 1e-3);
  const double d_stop = (v_norm * v_norm) / (2.0 * a_brake) + safety_margin_;

  std::cerr << "[COLLISION] vx=" << vx
            << " vy=" << vy
            << " wz=" << wz
            << " |v|=" << v_norm
            << " d_stop=" << d_stop << "\n";

  std::vector<double> min({
    static_cast<double>(-robot_radius_ - safety_margin_),
    static_cast<double>(-robot_radius_ - safety_margin_),
    static_cast<double>(z_min_filter_)});
  std::vector<double> max({
    static_cast<double>(robot_radius_ + safety_margin_ +
      std::max(0.0, v_norm * v_norm / (2.0 * std::max(brake_acc_, 1e-3)))),
    static_cast<double>(robot_radius_ + safety_margin_),
    static_cast<double>(robot_height_)});

  std::cerr << "[COLLISION] Filter box min=[" << min[0] << ", " << min[1] << ", " << min[2]
            << "] max=[" << max[0] << ", " << max[1] << ", " << max[2] << "]\n";

  const auto & cloud = PointPerceptionsOpsView(perceptions)
    .downsample(downsample_leaf_size_)
    .filter({-2.0, -2.0, z_min_filter_}, {2.0, 2.0, robot_height_})
    .fuse(motion_frame_)
    ->filter(min, max)
    .as_points();

  std::cerr << "[COLLISION] Cloud size after fuse/filter: "
            << cloud.size() << "\n";

  if (cloud.empty()) {
    publish_collision_zone_marker(min, max, cloud, imminent);
    return false;
  }

  geometry_msgs::msg::Pose base_pose;
  base_pose.orientation.w = 1.0;

  const double r = robot_radius_;
  const double dx = vx / v_norm;
  const double dy = vy / v_norm;
  const double r_sq = r * r;
  const double x_max = d_stop + r;

  std::cerr << "[COLLISION] Entering translation check dx=" << dx
            << " dy=" << dy
            << " r_sq=" << r_sq
            << " x_max=" << x_max << "\n";

  for (const auto & p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      std::cerr << "[COLLISION] Skipping invalid point (NaN/Inf) in LIN: ("
                << p.x << "," << p.y << "," << p.z << ")\n";
      continue;
    }

    const double px = p.x;
    const double py = p.y;

    const double x_prime =  dx * px + dy * py;
    const double y_prime = -dy * px + dx * py;


   std::cerr << "[COLLISION] Point: ("
                << p.x << "," << p.y << "," << p.z << ")\t"
                << "x_prime = " << x_prime << "\t"
                << "y_prime = " << y_prime << "\t"
                << std::endl;

    if (x_prime <= 0.0 || x_prime > x_max) {continue;}
    if ((y_prime * y_prime) > r_sq) {continue;}

    std::cerr << "[COLLISION] LIN hit: p=("
              << px << "," << py << "," << p.z
              << ") x'=" << x_prime
              << " y'=" << y_prime
              << "\n";

    imminent = true;
    publish_collision_zone_marker(min, max, cloud, imminent);
    return imminent;
  }

  std::cerr << "[COLLISION] Translation: no collision\n";
  publish_collision_zone_marker(min, max, cloud, imminent);
  return imminent;
}


void
ControllerMethodBase::publish_collision_zone_marker(
  const std::vector<double> & min,
  const std::vector<double> & max,
  const pcl::PointCloud<pcl::PointXYZ> & cloud,
  bool imminent_collision)
{
  if (!collision_marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;

  // 0) Borrar todo lo anterior en este namespace
  {
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = motion_frame_;
    clear.header.stamp = get_node()->now();
    clear.ns = "collision_zone";
    clear.id = 0;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);
  }

  const rclcpp::Time stamp = get_node()->now();

  // Color según colisión inminente
  std_msgs::msg::ColorRGBA color;
  color.r = imminent_collision ? 1.0f : 0.0f;
  color.g = imminent_collision ? 0.0f : 1.0f;
  color.b = 0.0f;
  color.a = 0.25f;

  // 1) CUBO que representa la caja [min, max] usada en el filtro
  {
    visualization_msgs::msg::Marker box;
    box.header.frame_id = motion_frame_;
    box.header.stamp = stamp;
    box.ns = "collision_zone";
    box.id = 1;
    box.type = visualization_msgs::msg::Marker::CUBE;
    box.action = visualization_msgs::msg::Marker::ADD;

    // Centro y tamaño del cubo
    const double cx = 0.5 * (min[0] + max[0]);
    const double cy = 0.5 * (min[1] + max[1]);
    const double cz = 0.5 * (min[2] + max[2]);

    const double sx = (max[0] - min[0]);
    const double sy = (max[1] - min[1]);
    const double sz = (max[2] - min[2]);

    box.pose.position.x = cx;
    box.pose.position.y = cy;
    box.pose.position.z = cz;
    box.pose.orientation.w = 1.0;

    box.scale.x = sx;
    box.scale.y = sy;
    box.scale.z = sz;

    box.color = color;
    box.lifetime = rclcpp::Duration(0, 200 * 1000000);  // 0.2s

    array.markers.push_back(box);
  }

  // 2) PUNTOS: la nube filtrada que realmente se usa en la detección
  {
    visualization_msgs::msg::Marker pts;
    pts.header.frame_id = motion_frame_;
    pts.header.stamp = stamp;
    pts.ns = "collision_zone";
    pts.id = 2;
    pts.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    pts.action = visualization_msgs::msg::Marker::ADD;

    pts.pose.orientation.w = 1.0;  // sin transformación extra

    // Tamaño de cada esfera/punto
    const float point_scale = 0.03f;
    pts.scale.x = point_scale;
    pts.scale.y = point_scale;
    pts.scale.z = point_scale;

    // Color igual que el cubo (verde/rojo según colisión)
    pts.color = color;
    pts.lifetime = rclcpp::Duration(0, 200 * 1000000);

    pts.points.reserve(cloud.points.size());
    for (const auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      geometry_msgs::msg::Point gp;
      gp.x = p.x;
      gp.y = p.y;
      gp.z = p.z;
      pts.points.push_back(gp);
    }

    array.markers.push_back(pts);
  }

  collision_marker_pub_->publish(array);
}

}  // namespace easynav
