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

#include "gtest/gtest.h"

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

  bool was_on_initialize_called() const
  {
    return on_initialize_called_;
  }

private:
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
    odom_.header.frame_id = get_tf_prefix() + "base_link";
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

  method.initialize(node, "test");

  EXPECT_EQ(method.get_node(), node) << "initialize() should set parent_node_ correctly.";
}

TEST_F(CoreMethodTestCase, OnInitializeCalled)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  MockMethod method;

  method.initialize(node, "test");

  EXPECT_TRUE(method.was_on_initialize_called()) <<
    "on_initialize() should be called during initialization.";
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
