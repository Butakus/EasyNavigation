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

/// \file
/// \brief Tests that verify the pluginlib plugin registration, loading, and
///        correctness of all built-in PerceptionHandler plugins.

#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_loader.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_sensors/SensorsNode.hpp"
#include "easynav_sensors/types/Perceptions.hpp"
#include "easynav_sensors/types/PointPerception.hpp"
#include "easynav_sensors/types/IMUPerception.hpp"
#include "easynav_sensors/types/GNSSPerception.hpp"
#include "easynav_sensors/types/ImagePerception.hpp"
#include "easynav_sensors/types/DetectionsPerception.hpp"
#include "easynav_common/types/NavState.hpp"

#include "gtest/gtest.h"

using easynav::PerceptionHandler;
using easynav::PerceptionPtr;

/// All plugin names that must be registered in the package plugin XML.
static const std::vector<std::string> kAllPlugins = {
  "easynav_sensors/PointPerceptionHandler",
  "easynav_sensors/IMUPerceptionHandler",
  "easynav_sensors/GNSSPerceptionHandler",
  "easynav_sensors/ImagePerceptionHandler",
  "easynav_sensors/DetectionsPerceptionHandler",
};

class PluginTestCase : public ::testing::Test
{
protected:
  ~PluginTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    loader_ = std::make_unique<pluginlib::ClassLoader<PerceptionHandler>>(
      "easynav_sensors", "easynav::PerceptionHandler");
    node_ = rclcpp_lifecycle::LifecycleNode::make_shared("plugin_test_node");
  }

  void TearDown() override {}

  /// Helper: load and initialise a plugin by name.
  std::shared_ptr<PerceptionHandler> load(const std::string & plugin_name)
  {
    auto handler = loader_->createSharedInstance(plugin_name);
    handler->initialize(node_, "test_sensor");
    return handler;
  }

  /// Helper: call populate_nav_state and verify that the expected collection
  /// type \p C was stored under \p group with exactly one element.
  template<typename C>
  void assert_populate_nav_state(std::shared_ptr<PerceptionHandler> handler)
  {
    auto perception = handler->create();
    ASSERT_NE(perception, nullptr);

    std::vector<PerceptionPtr> perceptions = {{perception, nullptr}};
    easynav::NavState ns;
    const std::string group = handler->group();
    handler->populate_nav_state(group, perceptions, ns);

    // get<C> throws if the key is absent — let it propagate as a test failure.
    const auto & result = ns.get<C>(group);
    ASSERT_EQ(result.size(), 1u);
  }

  std::unique_ptr<pluginlib::ClassLoader<PerceptionHandler>> loader_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
};

// ---------------------------------------------------------------------------
// 1. All built-in plugins are registered and discoverable.
// ---------------------------------------------------------------------------

TEST_F(PluginTestCase, all_plugins_are_available)
{
  for (const auto & name : kAllPlugins) {
    SCOPED_TRACE("plugin: " + name);
    EXPECT_TRUE(loader_->isClassAvailable(name));
  }
}

// ---------------------------------------------------------------------------
// 2. Every plugin loads without exception, initialises, and returns the
//    expected group name.
// ---------------------------------------------------------------------------

TEST_F(PluginTestCase, plugins_load_and_return_correct_group)
{
  const std::vector<std::pair<std::string, std::string>> cases = {
    {"easynav_sensors/PointPerceptionHandler", "points"},
    {"easynav_sensors/IMUPerceptionHandler", "imu"},
    {"easynav_sensors/GNSSPerceptionHandler", "gnss"},
    {"easynav_sensors/ImagePerceptionHandler", "image"},
    {"easynav_sensors/DetectionsPerceptionHandler", "detections"},
  };

  for (const auto & [plugin, expected_group] : cases) {
    SCOPED_TRACE("plugin: " + plugin);
    std::shared_ptr<PerceptionHandler> handler;
    ASSERT_NO_THROW(handler = load(plugin));
    ASSERT_NE(handler, nullptr);
    EXPECT_EQ(handler->group(), expected_group);
    EXPECT_EQ(handler->get_sensor_name(), "test_sensor");
  }
}

// ---------------------------------------------------------------------------
// 3. create() returns a non-null PerceptionBase for every plugin.
// ---------------------------------------------------------------------------

TEST_F(PluginTestCase, plugins_create_non_null_perception)
{
  for (const auto & name : kAllPlugins) {
    SCOPED_TRACE("plugin: " + name);
    auto handler = load(name);
    std::shared_ptr<easynav::PerceptionBase> perception;
    ASSERT_NO_THROW(perception = handler->create());
    EXPECT_NE(perception, nullptr);
  }
}

// ---------------------------------------------------------------------------
// 4. populate_nav_state stores correctly-typed perceptions in NavState.
// ---------------------------------------------------------------------------

TEST_F(PluginTestCase, point_plugin_populates_nav_state)
{
  auto handler = load("easynav_sensors/PointPerceptionHandler");
  ASSERT_NO_THROW((assert_populate_nav_state<easynav::PointPerceptions>(handler)));
}

TEST_F(PluginTestCase, imu_plugin_populates_nav_state)
{
  auto handler = load("easynav_sensors/IMUPerceptionHandler");
  ASSERT_NO_THROW((assert_populate_nav_state<easynav::IMUPerceptions>(handler)));
}

TEST_F(PluginTestCase, gnss_plugin_populates_nav_state)
{
  auto handler = load("easynav_sensors/GNSSPerceptionHandler");
  ASSERT_NO_THROW((assert_populate_nav_state<easynav::GNSSPerceptions>(handler)));
}

TEST_F(PluginTestCase, image_plugin_populates_nav_state)
{
  auto handler = load("easynav_sensors/ImagePerceptionHandler");
  ASSERT_NO_THROW((assert_populate_nav_state<easynav::ImagePerceptions>(handler)));
}

TEST_F(PluginTestCase, detections_plugin_populates_nav_state)
{
  auto handler = load("easynav_sensors/DetectionsPerceptionHandler");
  ASSERT_NO_THROW((assert_populate_nav_state<easynav::DetectionsPerceptions>(handler)));
}

// ---------------------------------------------------------------------------
// 5. An explicit plugin on one sensor does NOT change the default for other
//    sensors of the same message type that omit the 'plugin:' parameter.
//    Verified through SensorsNode: configure sensorA with an explicit plugin,
//    then configure sensorB with only a type and confirm it still uses the
//    built-in default (PointPerceptionHandler).
// ---------------------------------------------------------------------------

TEST_F(PluginTestCase, explicit_plugin_does_not_override_default_for_other_sensors)
{
  auto sensors_node = easynav::SensorsNode::make_shared();

  // sensorA: explicit plugin (same as the built-in default for LaserScan)
  sensors_node->declare_parameter("sensorA.topic", std::string("/scanA"));
  sensors_node->declare_parameter("sensorA.type", std::string("sensor_msgs/msg/LaserScan"));
  sensors_node->declare_parameter("sensorA.plugin",
    std::string("easynav_sensors/PointPerceptionHandler"));

  // sensorB: same type, NO explicit plugin — must still resolve to built-in default
  sensors_node->declare_parameter("sensorB.topic", std::string("/scanB"));
  sensors_node->declare_parameter("sensorB.type", std::string("sensor_msgs/msg/LaserScan"));

  sensors_node->set_parameter({"sensors", std::vector<std::string>{"sensorA", "sensorB"}});

  // on_configure must succeed for both sensors without throwing
  ASSERT_NO_THROW(
    sensors_node->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

  EXPECT_EQ(
    sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}
