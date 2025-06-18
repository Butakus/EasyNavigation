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

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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
    ->as_points();

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
    ->as_points();

  ASSERT_EQ(fused.size(), 2u);

  EXPECT_FLOAT_EQ(fused[0].x, 2.0f);  // sensor +1.0
  EXPECT_FLOAT_EQ(fused[0].y, 2.0f);
  EXPECT_FLOAT_EQ(fused[0].z, 3.0f);

  EXPECT_FLOAT_EQ(fused[1].x, 0.0f);  // sensor2 -1.0
  EXPECT_FLOAT_EQ(fused[1].y, 2.0f);
  EXPECT_FLOAT_EQ(fused[1].z, 3.0f);
}
