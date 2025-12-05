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
#include <expected>

#include "nav_msgs/msg/odometry.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"
#include "easynav_core/LocalizerMethodBase.hpp"

class CoreMethodTestCase : public ::testing::Test
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

// Mock class to test the behaviour of MethodBase initialization
class MockMethod : public easynav::MethodBase
{
public:
  MockMethod() = default;
  ~MockMethod() = default;

  std::expected<void, std::string> on_initialize() override
  {
    on_initialize_called_ = true;
    return {};
  }

public:
  bool on_initialize_called_ {false};
};

/** Dummy method class to test behvaiour of derived methods */
class TestLocalizer : public easynav::LocalizerMethodBase
{
public:
  TestLocalizer() = default;
  ~TestLocalizer() = default;

  std::expected<void, std::string> on_initialize() override
  {
    odom_.header.frame_id = get_tf_info().robot_frame;
    odom_.pose.pose.position.x = 5;
    return {};
  }

  virtual void update_rt(easynav::NavState & nav_state) override
  {
    (void) nav_state;
    odom_.pose.pose.position.x = 10;
  }

  virtual void update(easynav::NavState & nav_state) override
  {
    (void) nav_state;
    odom_.pose.pose.position.x = 10;
  }

private:
  nav_msgs::msg::Odometry odom_ {};
};


TEST(MethodBaseTest, DefaultConstructor)
{
  easynav::MethodBase method;
  EXPECT_EQ(
    method.get_node(),
    nullptr
  ) << "Default constructor should initialize parent_node_ to nullptr.";
}

TEST_F(CoreMethodTestCase, InitializeSetsParentNode)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  easynav::MethodBase method;
  easynav::TFInfo tf_info;
  method.initialize(node, "test", tf_info);
}

TEST_F(CoreMethodTestCase, OnInitializeCalled)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  MockMethod method;
  easynav::TFInfo tf_info;
  method.initialize(node, "test", tf_info);
  EXPECT_TRUE(method.on_initialize_called_) <<
    "on_initialize() should be called during initialization.";
}

TEST(Core_MethodBase, TFInfoPropagatesToDerived)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_tfinfo_node");

  class TFInfoProbeMethod : public easynav::MethodBase
  {
public:
    std::expected<void, std::string> on_initialize() override
    {
      seen_tf_info = get_tf_info();
      return {};
    }

    easynav::TFInfo seen_tf_info;
  };

  TFInfoProbeMethod method;
  easynav::TFInfo tf_info;
  tf_info.tf_prefix = "robot_/";
  tf_info.map_frame = "world";
  tf_info.odom_frame = "world_odom";
  tf_info.robot_frame = "robot_base";

  method.initialize(node, "test_plugin", tf_info);

  EXPECT_EQ(method.seen_tf_info.tf_prefix, tf_info.tf_prefix);
  EXPECT_EQ(method.seen_tf_info.map_frame, tf_info.map_frame);
  EXPECT_EQ(method.seen_tf_info.odom_frame, tf_info.odom_frame);
  EXPECT_EQ(method.seen_tf_info.robot_frame, tf_info.robot_frame);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
