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
  parent_node->create_publisher<visualization_msgs::msg::MarkerArray>("collision_area", 10);

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
    .filter({-2.0, -2.0, -1.0}, {2.0, 2.0, 3.0})
    .fuse(motion_frame_)
    ->filter(min, max)
    .as_points();

  std::cerr << "[COLLISION] Cloud size after fuse/filter: "
            << cloud.size() << "\n";

  if (cloud.empty()) {return false;}

  //
  // Rotational branch
  //
  if (v_norm < linear_speed_min_threshold_ &&
      std::fabs(wz) > angular_speed_min_threshold_) {

    std::cerr << "[COLLISION] Entering rotation check: v_norm=" << v_norm
              << " wz=" << wz << "\n";

    const double r_max_rot = robot_radius_ + rot_safety_margin_;
    const double r_max_rot_sq = r_max_rot * r_max_rot;

    for (const auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        std::cerr << "[COLLISION] Skipping invalid point (NaN/Inf) in ROT: ("
                  << p.x << "," << p.y << "," << p.z << ")\n";
        continue;
      }

      if (p.z < z_min_filter_ || p.z > robot_height_) {continue;}

      const double r_sq = static_cast<double>(p.x) * p.x +
                          static_cast<double>(p.y) * p.y;

      if (r_sq <= r_max_rot_sq) {
        std::cerr << "[COLLISION] ROT hit at point (" << p.x << ", " << p.y << ", " << p.z
                  << ") r_sq=" << r_sq << " <= " << r_max_rot_sq << "\n";
        return true;
      }
    }

    std::cerr << "[COLLISION] Rotation: no collision.\n";
    return false;
  }

  //
  // Translational branch
  //
  if (v_norm < linear_speed_min_threshold_) {
    std::cerr << "[COLLISION] v_norm below threshold → no collision\n";
    return false;
  }

  const double r = robot_radius_;
  const double dx = vx / v_norm;
  const double dy = vy / v_norm;
  const double r_sq = r * r;
  const double x_max = d_stop + r;

  std::cerr << "[COLLISION] Entering translation check dx=" << dx
            << " dy=" << dy
            << " x_max=" << x_max << "\n";

  for (const auto & p : cloud.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      std::cerr << "[COLLISION] Skipping invalid point (NaN/Inf) in LIN: ("
                << p.x << "," << p.y << "," << p.z << ")\n";
      continue;
    }
    if (p.z < z_min_filter_ || p.z > robot_height_) {continue;}

    const double px = p.x;
    const double py = p.y;

    const double x_prime =  dx * px + dy * py;
    const double y_prime = -dy * px + dx * py;

    if (x_prime <= 0.0 || x_prime > x_max) {continue;}
    if ((y_prime * y_prime) > r_sq) {continue;}

    std::cerr << "[COLLISION] LIN hit: p=("
              << px << "," << py << "," << p.z
              << ") x'=" << x_prime
              << " y'=" << y_prime
              << "\n";
    return true;
  }

  std::cerr << "[COLLISION] Translation: no collision\n";
  return false;
}

void
ControllerMethodBase::publish_collision_zone_marker(
  const geometry_msgs::msg::Pose & base_pose,
  double vx, double vy, double wz,
  double d_stop,
  bool imminent_collision)
{
  if (!collision_marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray array;

  // Borramos todo lo anterior de este namespace
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

  // Color según si hay colisión inminente
  std_msgs::msg::ColorRGBA color;
  color.r = imminent_collision ? 1.0f : 0.0f;
  color.g = imminent_collision ? 0.0f : 1.0f;
  color.b = 0.0f;
  color.a = 0.25f;  // semitransparente

  const double v_norm = std::sqrt(vx * vx + vy * vy);

  // Altura exacta del volumen que usa is_inminent_collision
  const double z_min = z_min_filter_;
  const double z_max = robot_height_;
  const double z_center = 0.5 * (z_min + z_max);
  const double z_height = z_max - z_min;

  // ------------------------------------------------------------------
  // 1) Volumen de ROTACIÓN: disco/cilindro alrededor del robot
  // ------------------------------------------------------------------
  if (v_norm < linear_speed_min_threshold_ &&
      std::fabs(wz) > angular_speed_min_threshold_) {

    const double r_max_rot = robot_radius_ + rot_safety_margin_;

    visualization_msgs::msg::Marker m_rot;
    m_rot.header.frame_id = motion_frame_;
    m_rot.header.stamp = stamp;
    m_rot.ns = "collision_zone";
    m_rot.id = 10;
    m_rot.type = visualization_msgs::msg::Marker::CYLINDER;
    m_rot.action = visualization_msgs::msg::Marker::ADD;

    m_rot.pose = base_pose;
    m_rot.pose.position.z = z_center;  // centrado en la banda [z_min, z_max]

    // Sin rotación especial: el cilindro está alineado con z
    // (la orientación de base_pose ya incluye el yaw del robot si quieres)
    // m_rot.pose.orientation = base_pose.orientation;

    m_rot.scale.x = 2.0 * r_max_rot;
    m_rot.scale.y = 2.0 * r_max_rot;
    m_rot.scale.z = z_height;

    m_rot.color = color;
    m_rot.lifetime = rclcpp::Duration(0, 200 * 1000000);  // 0.2 s

    array.markers.push_back(m_rot);
  }

  // ------------------------------------------------------------------
  // 2) Volumen de TRASLACIÓN: cápsula alineada con la velocidad
  // ------------------------------------------------------------------
  if (v_norm >= linear_speed_min_threshold_) {
    const double r = robot_radius_;

    const double dx = vx / v_norm;
    const double dy = vy / v_norm;

    // longitud del cilindro (parte "recta" de la cápsula)
    const double length = d_stop;

    // 2.1. Parte cilíndrica
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = motion_frame_;
      m.header.stamp = stamp;
      m.ns = "collision_zone";
      m.id = 1;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;

      m.pose = base_pose;
      m.pose.position.x += dx * (length * 0.5);
      m.pose.position.y += dy * (length * 0.5);
      m.pose.position.z = z_center;

      const double yaw = std::atan2(dy, dx);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      m.pose.orientation = tf2::toMsg(q);

      m.scale.x = 2.0 * r;
      m.scale.y = 2.0 * r;
      m.scale.z = z_height;

      m.color = color;
      m.lifetime = rclcpp::Duration(0, 200 * 1000000);

      array.markers.push_back(m);
    }

    // 2.2. Casquete delantero (semiesfera)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = motion_frame_;
      m.header.stamp = stamp;
      m.ns = "collision_zone";
      m.id = 2;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;

      m.pose = base_pose;
      m.pose.position.x += dx * length;
      m.pose.position.y += dy * length;
      m.pose.position.z = z_center;

      m.scale.x = 2.0 * r;
      m.scale.y = 2.0 * r;
      m.scale.z = z_height;

      m.color = color;
      m.lifetime = rclcpp::Duration(0, 200 * 1000000);

      array.markers.push_back(m);
    }

    // Nota: no dibujamos esfera trasera para que el volumen
    // se parezca más a lo que realmente evalúa is_inminent_collision.
  }

  collision_marker_pub_->publish(array);
}


}  // namespace easynav
