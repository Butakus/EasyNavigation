// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in sh0rt)
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


#include "gtest/gtest.h"
#include <memory>
#include <vector>
#include <cmath>

#include "tf2_ros/transform_listener.hpp"

#include "easynav_common/types/PointPerception.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


class PerceptionsOpsTest : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};


TEST(PerceptionsOpsViewTests, FilterTest)
{
  easynav::PointPerceptions perceptions;

  {
    auto perception = std::make_shared<easynav::PointPerception>();

    perception->valid = true;
    for (float i = -1.0f; i <= 1.0f; i += 0.1f) {
      pcl::PointXYZ pt;
      pt.x = i;
      pt.y = i;
      pt.z = i;
      perception->data.push_back(pt);
    }

    perceptions.push_back(perception);
  }

  easynav::PointPerceptionsOpsView view(perceptions);

  std::vector<double> min_bounds = {0.0, 0.0, 0.0};
  std::vector<double> max_bounds = {0.5, 0.5, 0.5};
  view.filter(min_bounds, max_bounds);

  pcl::PointCloud<pcl::PointXYZ> result = view.as_points();
  for (const auto & pt : result.points) {
    EXPECT_GE(pt.x, 0.0f);
    EXPECT_LE(pt.x, 0.5f);
    EXPECT_GE(pt.y, 0.0f);
    EXPECT_LE(pt.y, 0.5f);
    EXPECT_GE(pt.z, 0.0f);
    EXPECT_LE(pt.z, 0.5f);
  }
}


TEST_F(PerceptionsOpsTest, CollapseTest)
{
  easynav::PointPerceptions perceptions;

  auto perception = std::make_shared<easynav::PointPerception>();

  perception->valid = true;
  perception->data.push_back(pcl::PointXYZ(1.0f, 2.0f, 0.9f));
  perception->data.push_back(pcl::PointXYZ(1.0f, 4.0f, 1.3f));

  perceptions.push_back(perception);

  pcl::PointCloud<pcl::PointXYZ> collapsed =
    easynav::PointPerceptionsOpsView(perceptions)
    .collapse({NAN, NAN, 0.5})
    .as_points();

  EXPECT_EQ(collapsed.size(), 2u);
  for (const auto & pt : collapsed.points) {
    EXPECT_FLOAT_EQ(pt.x, 1.0f);
    EXPECT_FLOAT_EQ(pt.z, 0.5f);
  }

  EXPECT_EQ(perception->data.size(), 2u);
  EXPECT_FLOAT_EQ(perception->data[0].x, 1.0f);
  EXPECT_FLOAT_EQ(perception->data[0].y, 2.0f);
  EXPECT_FLOAT_EQ(perception->data[0].z, 0.9f);
  EXPECT_FLOAT_EQ(perception->data[1].x, 1.0f);
  EXPECT_FLOAT_EQ(perception->data[1].y, 4.0f);
  EXPECT_FLOAT_EQ(perception->data[1].z, 1.3f);
}

TEST(PerceptionsOpsViewTests, DownsampleTest)
{
  easynav::PointPerceptions perceptions;

  {
    auto perception = std::make_shared<easynav::PointPerception>();

    perception->valid = true;
    for (float i = 0.0; i < 1.0; i += 0.1f) {
      pcl::PointXYZ pt;
      pt.x = i;
      pt.y = 0.0f;
      pt.z = 0.0f;
      perception->data.push_back(pt);
    }
    perceptions.push_back(perception);
  }

  easynav::PointPerceptionsOpsView view(perceptions);
  view.downsample(0.2);

  pcl::PointCloud<pcl::PointXYZ> result = view.as_points();
  EXPECT_LE(result.size(), 6u);
}


/// brief Fuse test (basic TF transform with identity)
TEST_F(PerceptionsOpsTest, FuseOperation)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_fuse_node");
  auto tf_buffer = easynav::RTTFBuffer::getInstance(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  rclcpp::Time stamp = node->now();

  geometry_msgs::msg::TransformStamped tf1;
  tf1.header.stamp = stamp;
  tf1.header.frame_id = "odom";
  tf1.child_frame_id = "sensor";
  tf1.transform.translation.x = 1.0;
  tf1.transform.rotation.w = 1.0;

  geometry_msgs::msg::TransformStamped tf2;
  tf2.header.stamp = stamp;
  tf2.header.frame_id = "odom";
  tf2.child_frame_id = "sensor2";
  tf2.transform.translation.x = -1.0;
  tf2.transform.rotation.w = 1.0;

  tf_buffer->setTransform(tf1, "default_authority", false);
  tf_buffer->setTransform(tf2, "default_authority", false);

  easynav::PointPerceptions perceptions;

  {
    auto perception = std::make_shared<easynav::PointPerception>();

    perception->valid = true;
    perception->stamp = stamp;
    perception->frame_id = "sensor";
    perception->data.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));

    perceptions.push_back(perception);
  }

  {
    auto perception = std::make_shared<easynav::PointPerception>();

    perception->valid = true;
    perception->stamp = stamp;
    perception->frame_id = "sensor2";
    perception->data.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));

    perceptions.push_back(perception);
  }

  pcl::PointCloud<pcl::PointXYZ> fused =
    easynav::PointPerceptionsOpsView(perceptions)
    .fuse("odom")
    .as_points();

  ASSERT_EQ(fused.size(), 2u);

  EXPECT_FLOAT_EQ(fused[0].x, 2.0f);  // sensor +1.0
  EXPECT_FLOAT_EQ(fused[0].y, 2.0f);
  EXPECT_FLOAT_EQ(fused[0].z, 3.0f);

  EXPECT_FLOAT_EQ(fused[1].x, 0.0f);  // sensor2 -1.0
  EXPECT_FLOAT_EQ(fused[1].y, 2.0f);
  EXPECT_FLOAT_EQ(fused[1].z, 3.0f);
}


TEST(PerceptionsOpsViewCtor, FromSinglePerception_Basic)
{
  easynav::PointPerception p;
  p.valid = true;
  p.frame_id = "sensor";
  p.stamp = rclcpp::Time(123, 456);

  // Populate a simple cloud of test points
  for (int i = -5; i <= 5; ++i) {
    p.data.emplace_back(static_cast<float>(i) * 0.1f, 1.0f, 2.0f);
  }

  easynav::PointPerceptionsOpsView view(p);
  pcl::PointCloud<pcl::PointXYZ> out = view.as_points();

  ASSERT_EQ(out.size(), p.data.size());
  for (std::size_t i = 0; i < out.size(); ++i) {
    EXPECT_FLOAT_EQ(out[i].x, p.data[i].x);
    EXPECT_FLOAT_EQ(out[i].y, p.data[i].y);
    EXPECT_FLOAT_EQ(out[i].z, p.data[i].z);
  }
}

TEST(PerceptionsOpsViewCtor, FromSinglePerception_EmptyOrInvalid)
{
  // Empty but valid
  {
    easynav::PointPerception p;
    p.valid = true;
    easynav::PointPerceptionsOpsView view(p);
    EXPECT_EQ(view.as_points().size(), 0u);
  }

  // With data but invalid
  {
    easynav::PointPerception p;
    p.valid = false;
    p.data.emplace_back(1.f, 2.f, 3.f);
    easynav::PointPerceptionsOpsView view(p);
    EXPECT_EQ(view.as_points().size(), 0u);
  }
}

TEST(PerceptionsOpsViewCtor, FromSinglePerception_FilterDownsampleCollapse)
{
  easynav::PointPerception p;
  p.valid = true;

  // Fill with a 3x3x3 grid of points
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      for (int z = -1; z <= 1; ++z) {
        p.data.emplace_back(static_cast<float>(x),
                            static_cast<float>(y),
                            static_cast<float>(z));
      }
    }
}

  easynav::PointPerceptionsOpsView view(p);

  // Filter: keep points within [0, 1]
  view.filter({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0});
  auto filtered = view.as_points();
  ASSERT_EQ(filtered.size(), 8u);

  // Downsample: coarse resolution, should not increase count
  view.downsample(1.0);
  auto down = view.as_points();
  EXPECT_LE(down.size(), 8u);

  // Collapse: force Z = 0.25
  auto collapsed = easynav::PointPerceptionsOpsView(p)
    .filter({0.0, 0.0, 0.0}, {1.0, 1.0, 1.0})
    .collapse({NAN, NAN, 0.25})
    .as_points();
  ASSERT_EQ(collapsed.size(), 8u);
  for (const auto & pt : collapsed.points) {
    EXPECT_FLOAT_EQ(pt.z, 0.25f);
}
}

TEST(PerceptionsOpsViewCtor, FromOneOfManyPerceptions_UseOneAndOperate)
{
  easynav::PointPerceptions many;

  auto selected = std::make_shared<easynav::PointPerception>();
  selected->valid = true;
  selected->frame_id = "sensor1";
  selected->stamp = rclcpp::Time(2, 0);
  selected->data.clear();
  selected->data.emplace_back(0.f, 0.f, 0.f);
  selected->data.emplace_back(0.5f, 0.5f, 0.5f);
  selected->data.emplace_back(1.0f, 1.0f, 1.0f);
  many.push_back(selected);

  easynav::PointPerceptionsOpsView view(*selected);

  // Must contain the same data
  auto pts = view.as_points();
  ASSERT_EQ(pts.size(), selected->data.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    EXPECT_FLOAT_EQ(pts[i].x, selected->data[i].x);
}

  // Apply filter
  view.filter({0.25, 0.25, 0.25}, {0.75, 0.75, 0.75});
  pts = view.as_points();
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_FLOAT_EQ(pts[0].x, 0.5f);

  // Collapse test
  auto collapsed = easynav::PointPerceptionsOpsView(*selected)
    .filter({0.25, 0.25, 0.25}, {0.75, 0.75, 0.75})
    .collapse({NAN, NAN, 0.1})
    .as_points();
  ASSERT_EQ(collapsed.size(), 1u);
  EXPECT_FLOAT_EQ(collapsed[0].z, 0.1f);
}

TEST_F(PerceptionsOpsTest, FromSinglePerception_AddAndFuseWithTF)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ctor_add_fuse");
  auto tf_buffer = easynav::RTTFBuffer::getInstance(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);
  rclcpp::Time stamp = node->now();

  easynav::PointPerception base;
  base.valid = true;
  base.frame_id = "sensorA";
  base.stamp = stamp;
  base.data.emplace_back(1.f, 2.f, 3.f);

  easynav::PointPerceptionsOpsView view(base);

  // Add a second perception
  pcl::PointCloud<pcl::PointXYZ> other;
  other.emplace_back(10.f, 20.f, 30.f);
  auto & view2 = view.add(other, "sensorB", stamp);

  // Register TFs
  geometry_msgs::msg::TransformStamped tA, tB;
  tA.header.stamp = stamp;
  tA.header.frame_id = "odom";
  tA.child_frame_id = "sensorA";
  tA.transform.translation.x = 1.0;
  tA.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tA, "default_authority", false);

  tB.header.stamp = stamp;
  tB.header.frame_id = "odom";
  tB.child_frame_id = "sensorB";
  tB.transform.translation.x = -2.0;
  tB.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tB, "default_authority", false);

  // Fuse to "odom" and check both transformed points
  auto fused_pts = view2.fuse("odom").as_points();
  ASSERT_EQ(fused_pts.size(), 2u);
  EXPECT_FLOAT_EQ(fused_pts[0].x, 2.0f);
  EXPECT_FLOAT_EQ(fused_pts[1].x, 8.0f);
}

TEST_F(PerceptionsOpsTest, FromSinglePerception_AddAndFuseWithTFDense)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ctor_add_fuse");
  auto tf_buffer = easynav::RTTFBuffer::getInstance(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);
  rclcpp::Time stamp = node->now();

  easynav::PointPerception base;
  base.valid = true;
  base.frame_id = "sensorA";
  base.stamp = stamp;
  base.data.emplace_back(1.f, 2.f, 3.f);

  easynav::PointPerceptionsOpsView view(base);

  // Add a second perception
  pcl::PointCloud<pcl::PointXYZ> other;
  other.emplace_back(10.f, 20.f, 30.f);

  // Add a third perception
  pcl::PointCloud<pcl::PointXYZ> dense_cloud;
  dense_cloud.reserve(250000);
  const int rings = 128;
  const int points_per_ring = 2000;
  for (int r = 0; r < rings; r++) {
    float elev = -15.0f + 30.0f * (float(r) / float(rings));
    float elev_rad = elev * M_PI / 180.0f;

    for (int i = 0; i < points_per_ring; i++) {
      float azim = float(i) * 0.18f;
      float azim_rad = azim * M_PI / 180.0f;

      float dist = 10.0f + 5.0f * std::sin(i * 0.002f);

      dense_cloud.emplace_back(
        dist * std::cos(elev_rad) * std::cos(azim_rad),
        dist * std::cos(elev_rad) * std::sin(azim_rad),
        dist * std::sin(elev_rad));
    }
  }

  // Add a fourth perception
  pcl::PointCloud<pcl::PointXYZ> dense_cloud_2;
  dense_cloud_2.reserve(250000);
  for (int r = 0; r < rings; r++) {
    float elev = -15.0f + 30.0f * (float(r) / float(rings));
    float elev_rad = elev * M_PI / 180.0f;

    for (int i = 0; i < points_per_ring; i++) {
      float azim = float(i) * 0.18f;
      float azim_rad = azim * M_PI / 180.0f;

      float dist = 10.0f + 5.0f * std::sin(i * 0.002f);

      dense_cloud_2.emplace_back(
        dist * std::cos(elev_rad) * std::cos(azim_rad),
        dist * std::cos(elev_rad) * std::sin(azim_rad),
        dist * std::sin(elev_rad));
    }
  }

  view.add(other, "sensorB", stamp)
  .add(dense_cloud, "sensorC", stamp)
  .add(dense_cloud_2, "sensorD", stamp);

  // Register TFs
  geometry_msgs::msg::TransformStamped tA, tB, tC;
  tA.header.stamp = stamp;
  tA.header.frame_id = "odom";
  tA.child_frame_id = "sensorA";
  tA.transform.translation.x = 1.0;
  tA.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tA, "default_authority", false);

  tB.header.stamp = stamp;
  tB.header.frame_id = "odom";
  tB.child_frame_id = "sensorB";
  tB.transform.translation.x = -2.0;
  tB.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tB, "default_authority", false);

  tC.header.stamp = stamp;
  tC.header.frame_id = "odom";
  tC.child_frame_id = "sensorC";
  tC.transform.translation.y = 1.5;
  tC.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tC, "default_authority", false);

  tC.header.stamp = stamp;
  tC.header.frame_id = "odom";
  tC.child_frame_id = "sensorD";
  tC.transform.translation.y = -1.0;
  tC.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tC, "default_authority", false);


  // Fuse to "odom" and check both transformed points
  auto t_start = std::chrono::steady_clock::now();
  auto fused_pts = view.fuse("odom").as_points();
  auto t_end = std::chrono::steady_clock::now();

  double ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();

  RCLCPP_INFO(
    node->get_logger(),
    "Fuse() executed in %.2f ms with ~%zu points",
    ms, fused_pts.size());

  ASSERT_EQ(fused_pts.size(), dense_cloud.size() + dense_cloud_2.size() + 2);

  EXPECT_FLOAT_EQ(fused_pts[0].x, 2.0f);  // 1 + 1
  EXPECT_FLOAT_EQ(fused_pts[1].x, 8.0f);  // 10 - 2
}

TEST_F(PerceptionsOpsTest, CollapseDenseLidarPerformance)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_collapse_dense_lidar");

  easynav::PointPerceptions perceptions;
  auto perception = std::make_shared<easynav::PointPerception>();

  perception->valid = true;
  perception->frame_id = "sensor_lidar";
  perception->stamp = node->now();

  pcl::PointCloud<pcl::PointXYZ> dense_cloud;
  dense_cloud.reserve(250000);

  const int rings = 128;
  const int points_per_ring = 2000;
  for (int r = 0; r < rings; ++r) {
    float elev_deg = -15.0f + 30.0f * (static_cast<float>(r) / static_cast<float>(rings));
    float elev_rad = elev_deg * static_cast<float>(M_PI) / 180.0f;

    for (int i = 0; i < points_per_ring; ++i) {
      float azim_deg = static_cast<float>(i) * 0.18f;
      float azim_rad = azim_deg * static_cast<float>(M_PI) / 180.0f;

      float dist = 10.0f + 5.0f * std::sin(static_cast<float>(i) * 0.002f);

      dense_cloud.emplace_back(
        dist * std::cos(elev_rad) * std::cos(azim_rad),
        dist * std::cos(elev_rad) * std::sin(azim_rad),
        dist * std::sin(elev_rad));
    }
  }

  perception->data = dense_cloud;
  perceptions.push_back(perception);

  auto t_start = std::chrono::steady_clock::now();
  pcl::PointCloud<pcl::PointXYZ> collapsed =
    easynav::PointPerceptionsOpsView(perceptions)
    .collapse({NAN, NAN, 0.5})
    .as_points();
  auto t_end = std::chrono::steady_clock::now();

  double ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();

  RCLCPP_INFO(
    node->get_logger(),
    "Collapse() executed in %.2f ms with %zu input points and %zu output points",
    ms,
    dense_cloud.size(),
    static_cast<std::size_t>(collapsed.size()));

  EXPECT_EQ(collapsed.size(), dense_cloud.size());

  ASSERT_FALSE(collapsed.empty());

  const auto & src0 = dense_cloud.points[0];
  const auto & dst0 = collapsed.points[0];
  EXPECT_FLOAT_EQ(dst0.x, src0.x);
  EXPECT_FLOAT_EQ(dst0.y, src0.y);
  EXPECT_FLOAT_EQ(dst0.z, 0.5f);

  const std::size_t mid_idx = collapsed.size() / 2;
  const auto & src_mid = dense_cloud.points[mid_idx];
  const auto & dst_mid = collapsed.points[mid_idx];
  EXPECT_FLOAT_EQ(dst_mid.x, src_mid.x);
  EXPECT_FLOAT_EQ(dst_mid.y, src_mid.y);
  EXPECT_FLOAT_EQ(dst_mid.z, 0.5f);

  const std::size_t last_idx = collapsed.size() - 1;
  const auto & src_last = dense_cloud.points[last_idx];
  const auto & dst_last = collapsed.points[last_idx];
  EXPECT_FLOAT_EQ(dst_last.x, src_last.x);
  EXPECT_FLOAT_EQ(dst_last.y, src_last.y);
  EXPECT_FLOAT_EQ(dst_last.z, 0.5f);

  EXPECT_EQ(perception->data.size(), dense_cloud.size());
  EXPECT_FLOAT_EQ(perception->data[0].x, dense_cloud[0].x);
  EXPECT_FLOAT_EQ(perception->data[0].y, dense_cloud[0].y);
  EXPECT_FLOAT_EQ(perception->data[0].z, dense_cloud[0].z);
}


TEST_F(PerceptionsOpsTest, All_pipeline)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_ctor_add_fuse");
  auto tf_buffer = easynav::RTTFBuffer::getInstance(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);
  rclcpp::Time stamp = node->now();

  easynav::PointPerception base;
  base.valid = true;
  base.frame_id = "sensorA";
  base.stamp = stamp;
  base.data.emplace_back(1.f, 2.f, 3.f);

  easynav::PointPerceptionsOpsView view(base);

  // Add a second perception
  pcl::PointCloud<pcl::PointXYZ> other;
  other.emplace_back(10.f, 20.f, 30.f);

  // Add a third perception
  pcl::PointCloud<pcl::PointXYZ> dense_cloud;
  dense_cloud.reserve(250000);
  const int rings = 128;
  const int points_per_ring = 2000;
  for (int r = 0; r < rings; r++) {
    float elev = -15.0f + 30.0f * (float(r) / float(rings));
    float elev_rad = elev * M_PI / 180.0f;

    for (int i = 0; i < points_per_ring; i++) {
      float azim = float(i) * 0.18f;
      float azim_rad = azim * M_PI / 180.0f;

      float dist = 10.0f + 5.0f * std::sin(i * 0.002f);

      dense_cloud.emplace_back(
        dist * std::cos(elev_rad) * std::cos(azim_rad),
        dist * std::cos(elev_rad) * std::sin(azim_rad),
        dist * std::sin(elev_rad));
    }
  }

  // Add a fourth perception
  pcl::PointCloud<pcl::PointXYZ> dense_cloud_2;
  dense_cloud_2.reserve(250000);
  for (int r = 0; r < rings; r++) {
    float elev = -15.0f + 30.0f * (float(r) / float(rings));
    float elev_rad = elev * M_PI / 180.0f;

    for (int i = 0; i < points_per_ring; i++) {
      float azim = float(i) * 0.18f;
      float azim_rad = azim * M_PI / 180.0f;

      float dist = 10.0f + 5.0f * std::sin(i * 0.002f);

      dense_cloud_2.emplace_back(
        dist * std::cos(elev_rad) * std::cos(azim_rad),
        dist * std::cos(elev_rad) * std::sin(azim_rad),
        dist * std::sin(elev_rad));
    }
  }

  view.add(other, "sensorB", stamp)
  .add(dense_cloud, "sensorC", stamp)
  .add(dense_cloud_2, "sensorD", stamp);

  // Register TFs
  geometry_msgs::msg::TransformStamped tA, tB, tC;
  tA.header.stamp = stamp;
  tA.header.frame_id = "odom";
  tA.child_frame_id = "sensorA";
  tA.transform.translation.x = 1.0;
  tA.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tA, "default_authority", false);

  tB.header.stamp = stamp;
  tB.header.frame_id = "odom";
  tB.child_frame_id = "sensorB";
  tB.transform.translation.x = -2.0;
  tB.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tB, "default_authority", false);

  tC.header.stamp = stamp;
  tC.header.frame_id = "odom";
  tC.child_frame_id = "sensorC";
  tC.transform.translation.y = 1.5;
  tC.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tC, "default_authority", false);

  tC.header.stamp = stamp;
  tC.header.frame_id = "odom";
  tC.child_frame_id = "sensorD";
  tC.transform.translation.y = -1.0;
  tC.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tC, "default_authority", false);


  // Fuse to "odom" and check both transformed points
  auto t_start = std::chrono::steady_clock::now();
  auto fused_pts = view
    .downsample(0.1)
    .filter({NAN, NAN, 1.0}, {NAN, NAN, 2.0})
    .fuse("odom")
    .filter({0.0, NAN, NAN}, {NAN, NAN, NAN})
    .collapse({NAN, NAN, 1.0})
    .as_points();
  auto t_end = std::chrono::steady_clock::now();

  double ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();

  RCLCPP_INFO(
    node->get_logger(),
    "All pipeline executed in %.2f ms with ~%zu points",
    ms, fused_pts.size());
}

TEST(PerceptionsOpsViewTests, CollapseEager_OwningView_ModifiesOwnedData)
{
  easynav::PointPerception p;
  p.valid = true;
  p.frame_id = "sensor";
  p.stamp = rclcpp::Time(0, 0);
  p.data.emplace_back(1.0f, 2.0f, 0.9f);
  p.data.emplace_back(1.0f, 4.0f, 1.3f);

  easynav::PointPerceptionsOpsView view(p);

  view.collapse({NAN, NAN, 0.5}, /*lazy=*/false);

  auto collapsed = view.as_points();
  ASSERT_EQ(collapsed.size(), 2u);
  EXPECT_FLOAT_EQ(collapsed[0].x, 1.0f);
  EXPECT_FLOAT_EQ(collapsed[0].z, 0.5f);
  EXPECT_FLOAT_EQ(collapsed[1].x, 1.0f);
  EXPECT_FLOAT_EQ(collapsed[1].z, 0.5f);

  const auto & owned_perceptions = view.get_perceptions();
  ASSERT_EQ(owned_perceptions.size(), 1u);
  ASSERT_TRUE(owned_perceptions[0]);
  EXPECT_FLOAT_EQ(owned_perceptions[0]->data[0].z, 0.5f);
  EXPECT_FLOAT_EQ(owned_perceptions[0]->data[1].z, 0.5f);

  EXPECT_FLOAT_EQ(p.data[0].z, 0.9f);
  EXPECT_FLOAT_EQ(p.data[1].z, 1.3f);
}

TEST(PerceptionsOpsViewTests, CollapseEager_NonOwningView_Ignored)
{
  easynav::PointPerceptions perceptions;
  auto perception = std::make_shared<easynav::PointPerception>();

  perception->valid = true;
  perception->frame_id = "sensor";
  perception->stamp = rclcpp::Time(0, 0);
  perception->data.emplace_back(1.0f, 2.0f, 0.9f);
  perception->data.emplace_back(1.0f, 4.0f, 1.3f);
  perceptions.push_back(perception);

  easynav::PointPerceptionsOpsView view(perceptions);

  view.collapse({NAN, NAN, 0.5}, /*lazy=*/false);

  auto out = view.as_points();
  ASSERT_EQ(out.size(), 2u);
  EXPECT_FLOAT_EQ(out[0].z, 0.9f);
  EXPECT_FLOAT_EQ(out[1].z, 1.3f);

  EXPECT_FLOAT_EQ(perception->data[0].z, 0.9f);
  EXPECT_FLOAT_EQ(perception->data[1].z, 1.3f);
}

TEST_F(PerceptionsOpsTest, FilterPostFuse_Lazy_AppliesInTargetFrame)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_filter_postfuse_lazy");
  auto tf_buffer = easynav::RTTFBuffer::getInstance(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  rclcpp::Time stamp = node->now();

  // TF: odom -> sensor (x + 1.0)
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "sensor";
  tf.transform.translation.x = 1.0;
  tf.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tf, "default_authority", false);

  easynav::PointPerception p;
  p.valid = true;
  p.frame_id = "sensor";
  p.stamp = stamp;
  // In "sensor"
  p.data.emplace_back(0.0f, 0.0f, 0.0f);     // in odom -> (1, 0, 0)
  p.data.emplace_back(-0.75f, 0.0f, 0.0f);   // in odom -> (0.25, 0, 0)

  easynav::PointPerceptionsOpsView view(p);

  // fuse + lazy filter in odom: x >= 0.5
  auto cloud = view
    .fuse("odom")
    .filter({0.5, NAN, NAN}, {NAN, NAN, NAN})  // lazy_post_fuse by default = true
    .as_points();

  ASSERT_EQ(cloud.size(), 1u);
  EXPECT_FLOAT_EQ(cloud[0].x, 1.0f);  // onlsy the first point will survice (x=1 in odom)
}

TEST_F(PerceptionsOpsTest, FilterPostFuse_Eager_AppliesInTargetFrame)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_filter_postfuse_eager");
  auto tf_buffer = easynav::RTTFBuffer::getInstance(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  rclcpp::Time stamp = node->now();

  // TF: odom -> sensor (x + 1.0)
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "sensor";
  tf.transform.translation.x = 1.0;
  tf.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tf, "default_authority", false);

  easynav::PointPerception p;
  p.valid = true;
  p.frame_id = "sensor";
  p.stamp = stamp;
  p.data.emplace_back(0.0f, 0.0f, 0.0f);     // -> (1, 0, 0) in odom
  p.data.emplace_back(-0.75f, 0.0f, 0.0f);   // -> (0.25, 0, 0) un odom

  easynav::PointPerceptionsOpsView view(p);

  auto cloud = view
    .fuse("odom")
    .filter({0.5, NAN, NAN}, {NAN, NAN, NAN}, /*lazy_post_fuse=*/false)
    .as_points();

  ASSERT_EQ(cloud.size(), 1u);
  EXPECT_FLOAT_EQ(cloud[0].x, 1.0f);
}

TEST_F(PerceptionsOpsTest, Filter_PreAndPostFuse_EagerCombination)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_filter_pre_post_fuse");
  auto tf_buffer = easynav::RTTFBuffer::getInstance(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer);

  rclcpp::Time stamp = node->now();

  // TF: odom -> sensor (x + 1.0)
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "sensor";
  tf.transform.translation.x = 1.0;
  tf.transform.rotation.w = 1.0;
  tf_buffer->setTransform(tf, "default_authority", false);

  easynav::PointPerception p;
  p.valid = true;
  p.frame_id = "sensor";
  p.stamp = stamp;
  p.data.emplace_back(-1.0f, 0.0f, 0.0f);  // will disrcard in pre-fuse filter
  p.data.emplace_back(0.0f, 0.0f, 0.0f);   // -> (1,0,0) in odom
  p.data.emplace_back(1.0f, 0.0f, 0.0f);   // -> (2,0,0) in odom

  easynav::PointPerceptionsOpsView view(p);

  auto cloud = view
    // pre-fuse filter in frame "sensor": x in [0, 1]
    .filter({0.0, NAN, NAN}, {1.0, NAN, NAN})
    // Fuse to "odom"
    .fuse("odom")
    // post-fuse eager filter: x >= 1.5 in odom
    .filter({1.5, NAN, NAN}, {NAN, NAN, NAN}, /*lazy_post_fuse=*/false)
    .as_points();

  ASSERT_EQ(cloud.size(), 1u);
  EXPECT_FLOAT_EQ(cloud[0].x, 2.0f);  // only the original point x=1 in sensor survives
}

TEST(PerceptionsOpsViewTests, CollapseLazy_DoesNotAffectIntermediateFilter)
{
  easynav::PointPerception p;
  p.valid = true;
  p.frame_id = "sensor";
  p.stamp = rclcpp::Time(0, 0);
  p.data.emplace_back(1.0f, 2.0f, 0.9f);
  p.data.emplace_back(1.0f, 4.0f, 1.3f);

  easynav::PointPerceptionsOpsView view(p);

  // Collapse lazy Z=1.0
  view.collapse({NAN, NAN, 1.0}, /*lazy=*/true);

  // Filter in z [1.0, 1.0] in sensor frame: applied to original z (0.9 and 1.3),
  // so both points are discarded.
  view.filter({NAN, NAN, 1.0}, {NAN, NAN, 1.0});

  auto cloud = view.as_points();
  EXPECT_EQ(cloud.size(), 0u);
}
