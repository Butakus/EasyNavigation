// Copyright 2025 Intelligent Robotics Lab
//
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

#include "easynav_sensors/SensorsNode.hpp"
#include "easynav_common/types/NavState.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "gtest/gtest.h"


class SensorsLifecycleTestCase : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Basic node properties
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SensorsLifecycleTestCase, NodeNameIsCorrect)
{
  auto node = easynav::SensorsNode::make_shared();
  EXPECT_EQ(std::string(node->get_name()), "sensors_node");
}

TEST_F(SensorsLifecycleTestCase, GetRealTimeCbgNotNull)
{
  auto node = easynav::SensorsNode::make_shared();
  EXPECT_NE(node->get_real_time_cbg(), nullptr);
}

TEST_F(SensorsLifecycleTestCase, StartsInUnconfiguredState)
{
  auto node = easynav::SensorsNode::make_shared();
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle transitions with no sensors configured
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SensorsLifecycleTestCase, ConfigureWithNoSensors)
{
  auto node = easynav::SensorsNode::make_shared();
  // Leave sensors param as default (empty list)
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SensorsLifecycleTestCase, ActivateFromInactive)
{
  auto node = easynav::SensorsNode::make_shared();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
}

TEST_F(SensorsLifecycleTestCase, DeactivateFromActive)
{
  auto node = easynav::SensorsNode::make_shared();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SensorsLifecycleTestCase, CleanupFromInactive)
{
  auto node = easynav::SensorsNode::make_shared();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(SensorsLifecycleTestCase, ShutdownFromInactive)
{
  auto node = easynav::SensorsNode::make_shared();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

TEST_F(SensorsLifecycleTestCase, ShutdownFromActive)
{
  auto node = easynav::SensorsNode::make_shared();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

TEST_F(SensorsLifecycleTestCase, ShutdownFromUnconfigured)
{
  auto node = easynav::SensorsNode::make_shared();
  node->trigger_transition(
    lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
}

TEST_F(SensorsLifecycleTestCase, FullActivateDeactivateCleanupCycle)
{
  auto node = easynav::SensorsNode::make_shared();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  ASSERT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

// ─────────────────────────────────────────────────────────────────────────────
// cycle_rt and cycle behaviour
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SensorsLifecycleTestCase, CycleRtReturnsFalseWhenNoNewData)
{
  auto node = easynav::SensorsNode::make_shared();
  // Configure and activate with no sensors → no subscriptions, no new data
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  auto nav_state = std::make_shared<easynav::NavState>();
  // With no sensors and no data published, cycle_rt should return false
  bool result = node->cycle_rt(nav_state, false);
  EXPECT_FALSE(result);
}

TEST_F(SensorsLifecycleTestCase, CycleRtWithTriggerReturnsFalseWhenNoPerceptions)
{
  auto node = easynav::SensorsNode::make_shared();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  auto nav_state = std::make_shared<easynav::NavState>();
  // trigger=true, but perceptions_ is empty → no new data → false
  bool result = node->cycle_rt(nav_state, true);
  EXPECT_FALSE(result);
}

TEST_F(SensorsLifecycleTestCase, CycleDoesNotCrashWhenNoSensors)
{
  auto node = easynav::SensorsNode::make_shared();
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  auto nav_state = std::make_shared<easynav::NavState>();
  EXPECT_NO_THROW(node->cycle(nav_state));
}

// ─────────────────────────────────────────────────────────────────────────────
// Configure with specific sensor types
// ─────────────────────────────────────────────────────────────────────────────

TEST_F(SensorsLifecycleTestCase, ConfigureWithLaserScanSensor)
{
  auto node = easynav::SensorsNode::make_shared();

  std::vector<std::string> sensors = {"laser1"};
  node->declare_parameter("laser1.topic", std::string("/test_scan"));
  node->declare_parameter("laser1.type", std::string("sensor_msgs/msg/LaserScan"));
  node->declare_parameter("laser1.group", std::string("points"));
  node->set_parameter({"sensors", sensors});
  node->set_parameter({"laser1.topic", std::string("/test_scan")});
  node->set_parameter({"laser1.type", std::string("sensor_msgs/msg/LaserScan")});
  node->set_parameter({"laser1.group", std::string("points")});

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SensorsLifecycleTestCase, ConfigureWithPointCloud2Sensor)
{
  auto node = easynav::SensorsNode::make_shared();

  std::vector<std::string> sensors = {"lidar3d"};
  node->declare_parameter("lidar3d.topic", std::string("/test_pc2"));
  node->declare_parameter("lidar3d.type", std::string("sensor_msgs/msg/PointCloud2"));
  node->declare_parameter("lidar3d.group", std::string("points"));
  node->set_parameter({"sensors", sensors});
  node->set_parameter({"lidar3d.topic", std::string("/test_pc2")});
  node->set_parameter({"lidar3d.type", std::string("sensor_msgs/msg/PointCloud2")});
  node->set_parameter({"lidar3d.group", std::string("points")});

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(SensorsLifecycleTestCase, ConfigureWithMultipleSensors)
{
  auto node = easynav::SensorsNode::make_shared();

  std::vector<std::string> sensors = {"scan_front", "scan_back"};
  node->declare_parameter("scan_front.topic", std::string("/front_scan"));
  node->declare_parameter("scan_front.type", std::string("sensor_msgs/msg/LaserScan"));
  node->declare_parameter("scan_front.group", std::string("points"));
  node->declare_parameter("scan_back.topic", std::string("/back_scan"));
  node->declare_parameter("scan_back.type", std::string("sensor_msgs/msg/LaserScan"));
  node->declare_parameter("scan_back.group", std::string("points"));

  node->set_parameter({"sensors", sensors});
  node->set_parameter({"scan_front.topic", std::string("/front_scan")});
  node->set_parameter({"scan_front.type", std::string("sensor_msgs/msg/LaserScan")});
  node->set_parameter({"scan_front.group", std::string("points")});
  node->set_parameter({"scan_back.topic", std::string("/back_scan")});
  node->set_parameter({"scan_back.type", std::string("sensor_msgs/msg/LaserScan")});
  node->set_parameter({"scan_back.group", std::string("points")});

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  EXPECT_EQ(
    node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
