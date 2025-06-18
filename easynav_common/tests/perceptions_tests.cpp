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

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"

#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/PointPerception.hpp"
#include "easynav_common/types/ImagePerception.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"
#include "pcl/common/transforms.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/transform_datatypes.hpp"

#include "gtest/gtest.h"


class PerceptionsTestCase : public ::testing::Test
{
protected:
  ~PerceptionsTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp()
  {
    if (!initialized) {
      rclcpp::init(0, nullptr);
      initialized = true;
    }
  }

  void TearDown()
  {
  }

  bool initialized {false};
};

sensor_msgs::msg::LaserScan get_scan_test_1(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, std::numeric_limits<float>::infinity());

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_2(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 0.0);

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_3(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser_1";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[4] = 0.3;

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_4(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser_2";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 4.0);
  ret.ranges[4] = 1.0;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_0(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = static_cast<float>(i);
    p.y = static_cast<float>(i);
    p.z = 0.0;
    pc.push_back(p);
  }

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_1(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = std::numeric_limits<float>::infinity();
    p.y = std::numeric_limits<float>::infinity();
    p.z = 0.0;
    pc.push_back(p);
  }

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_2(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    pc.push_back(p);
  }

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_3(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = 5.0;
    p.y = 5.0;
    p.z = 0.0;
    pc.push_back(p);
  }

  pc[4].x = 0.3;
  pc[4].y = 0.3;
  pc[4].z = 0.0;

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_4(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = 5.0;
    p.y = 5.0;
    p.z = 0.0;
    pc.push_back(p);
  }

  pc[4].x = 1.0;
  pc[4].y = 1.0;
  pc[4].z = 0.0;

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

using namespace std::chrono_literals;


TEST_F(PerceptionsTestCase, PointPerceptionHandlerWorks)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_handler_node");

  auto handler = std::make_shared<easynav::PointPerceptionHandler>();
  auto perception = handler->create("laser1");

  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sub = handler->create_subscription(
    *node,
    "/test_scan",
    "sensor_msgs/msg/LaserScan",
    perception,
    cb_group);

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub =
    node->create_publisher<sensor_msgs::msg::LaserScan>("/test_scan",
    rclcpp::SensorDataQoS().reliable());

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_callback_group(cb_group, node->get_node_base_interface());


  auto now = node->get_clock()->now();
  const auto & data = get_scan_test_2(now);
  laser_pub->publish(data);

  auto start = now;
  while (node->get_clock()->now() - start < 100ms) {
    exe.spin_some();
  }

  auto loaded = std::dynamic_pointer_cast<easynav::PointPerception>(perception);
  ASSERT_NE(loaded, nullptr);

  ASSERT_TRUE(loaded->valid);
  ASSERT_TRUE(loaded->new_data);
  ASSERT_EQ(loaded->data.size(), data.ranges.size());

  double angle = data.angle_min;
  for (size_t i = 0; i < 16; ++i) {
    ASSERT_NEAR(loaded->data[i].x, data.ranges[i] * std::cos(angle), 0.01);
    ASSERT_NEAR(loaded->data[i].y, data.ranges[i] * std::sin(angle), 0.01);
    ASSERT_NEAR(loaded->data[i].z, 0.0, 0.0001);

    angle += data.angle_increment;
  }
}

TEST_F(PerceptionsTestCase, PointPerceptionHandlerPC2Works)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_pc2_handler_node");

  auto handler = std::make_shared<easynav::PointPerceptionHandler>();
  auto perception = handler->create("lidar1");

  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sub = handler->create_subscription(
    *node,
    "/test_pc2",
    "sensor_msgs/msg/PointCloud2",
    perception,
    cb_group);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub =
    node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/test_pc2", rclcpp::SensorDataQoS().reliable());

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_callback_group(cb_group, node->get_node_base_interface());

  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.header.frame_id = "lidar_frame";
  pcl_cloud.width = 3;
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = true;

  pcl_cloud.points.push_back(pcl::PointXYZ{1.0, 2.0, 3.0});
  pcl_cloud.points.push_back(pcl::PointXYZ{-1.5, 0.0, 0.5});
  pcl_cloud.points.push_back(pcl::PointXYZ{0.0, -2.0, -3.0});

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(pcl_cloud, msg);
  msg.header.frame_id = "lidar_frame";
  msg.header.stamp = node->now();

  pub->publish(msg);

  auto start = node->now();
  while (node->now() - start < 100ms) {
    exe.spin_some();
  }

  auto loaded = std::dynamic_pointer_cast<easynav::PointPerception>(perception);
  ASSERT_NE(loaded, nullptr);
  ASSERT_TRUE(loaded->valid);
  ASSERT_TRUE(loaded->new_data);
  ASSERT_EQ(loaded->frame_id, "lidar_frame");
  ASSERT_EQ(loaded->data.size(), 3u);

  ASSERT_NEAR(loaded->data[0].x, 1.0, 0.001);
  ASSERT_NEAR(loaded->data[0].y, 2.0, 0.001);
  ASSERT_NEAR(loaded->data[0].z, 3.0, 0.001);

  ASSERT_NEAR(loaded->data[1].x, -1.5, 0.001);
  ASSERT_NEAR(loaded->data[1].y, 0.0, 0.001);
  ASSERT_NEAR(loaded->data[1].z, 0.5, 0.001);

  ASSERT_NEAR(loaded->data[2].x, 0.0, 0.001);
  ASSERT_NEAR(loaded->data[2].y, -2.0, 0.001);
  ASSERT_NEAR(loaded->data[2].z, -3.0, 0.001);
}


TEST_F(PerceptionsTestCase, ImagePerceptionHandlerWorks)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_image_handler_node");

  auto handler = std::make_shared<easynav::ImagePerceptionHandler>();
  auto perception = handler->create("camera1");

  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sub = handler->create_subscription(
    *node,
    "/test_image",
    "sensor_msgs/msg/Image",
    perception,
    cb_group);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub =
    node->create_publisher<sensor_msgs::msg::Image>(
    "/test_image", rclcpp::SensorDataQoS().reliable());

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_callback_group(cb_group, node->get_node_base_interface());

  cv::Mat test_img(2, 3, CV_8UC3, cv::Scalar(10, 20, 30));
  test_img.at<cv::Vec3b>(0, 1) = cv::Vec3b(100, 110, 120);

  auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", test_img).toImageMsg();
  msg->header.frame_id = "camera_link";
  msg->header.stamp = node->now();

  image_pub->publish(*msg);

  auto start = node->now();
  while (node->now() - start < 100ms) {
    exe.spin_some();
  }

  auto loaded = std::dynamic_pointer_cast<easynav::ImagePerception>(perception);
  ASSERT_NE(loaded, nullptr);
  ASSERT_TRUE(loaded->valid);
  ASSERT_TRUE(loaded->new_data);
  ASSERT_EQ(loaded->frame_id, "camera_link");
  ASSERT_EQ(loaded->data.cols, 3);
  ASSERT_EQ(loaded->data.rows, 2);
  ASSERT_EQ(loaded->data.type(), CV_8UC3);
  ASSERT_EQ(loaded->data.at<cv::Vec3b>(0, 0), cv::Vec3b(10, 20, 30));
  ASSERT_EQ(loaded->data.at<cv::Vec3b>(0, 1), cv::Vec3b(100, 110, 120));
}

class DummyPerception : public easynav::PerceptionBase
{
public:
  std::string content;
};

class DummyHandler : public easynav::PerceptionHandler
{
public:
  std::string group() const override {return "dummy";}

  std::shared_ptr<easynav::PerceptionBase> create(const std::string &) override
  {
    auto p = std::make_shared<DummyPerception>();
    p->content = "initialized";
    return p;
  }

  rclcpp::SubscriptionBase::SharedPtr create_subscription(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & topic,
    const std::string &,
    std::shared_ptr<easynav::PerceptionBase> target,
    rclcpp::CallbackGroup::SharedPtr cb_group)
  {
    auto options = rclcpp::SubscriptionOptions();
    options.callback_group = cb_group;

    return node.create_subscription<std_msgs::msg::String>(
      topic, rclcpp::QoS(1).reliable(),
      [target](const std_msgs::msg::String::SharedPtr msg)
      {
        auto p = std::dynamic_pointer_cast<DummyPerception>(target);
        p->content = msg->data;
        p->valid = true;
        p->new_data = true;
      },
      options);
  }
};

TEST_F(PerceptionsTestCase, CustomPerceptionHandlerCanBeRegistered)
{
  auto node = rclcpp_lifecycle::LifecycleNode::make_shared("test_custom_handler_node");

  auto handler = std::make_shared<DummyHandler>();
  auto perception = handler->create("dummy1");

  auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto sub = handler->create_subscription(
    *node,
    "/dummy_topic",
    "std_msgs/msg/String",
    perception,
    cb_group);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub =
    node->create_publisher<std_msgs::msg::String>(
    "/dummy_topic", rclcpp::QoS(1).reliable());

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.add_callback_group(cb_group, node->get_node_base_interface());

  std_msgs::msg::String msg;
  msg.data = "HelloWorld";
  pub->publish(msg);

  auto start = node->now();
  while (node->now() - start < 100ms) {
    exe.spin_some();
  }

  auto loaded = std::dynamic_pointer_cast<DummyPerception>(perception);
  ASSERT_NE(loaded, nullptr);
  ASSERT_EQ(loaded->content, "HelloWorld");
  ASSERT_TRUE(loaded->valid);
}
