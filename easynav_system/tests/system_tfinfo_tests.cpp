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

#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_system/SystemNode.hpp"
#include "easynav_common/RTTFBuffer.hpp"

// We will use the existing dummy controller plugin
// (easynav::DummyController) which is already registered in
// easynav_controller_plugins.xml and uses get_tf_info().

using namespace std::chrono_literals;

class SystemTFInfoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    // Keep rclcpp up for other tests in this process.
  }
};


TEST_F(SystemTFInfoTest, SystemNodePropagatesFrameParametersNoNS)
{
  // Create a SystemNode as in production.
  auto system_node = std::make_shared<easynav::SystemNode>();

  // Set high-level frame parameters.
  const std::string map_frame = "my_map";
  const std::string odom_frame = "my_odom";
  const std::string robot_frame = "my_base";
  const std::string world_frame = "my_earth";
  const std::string tf_prefix = "";

  // NOTE: we only set parameters on SystemNode; the ControllerNode will
  // declare and read them during its own on_configure() call triggered by
  // SystemNode::on_configure().
  system_node->set_parameter({"tf_prefix", tf_prefix});
  system_node->set_parameter({"robot_frame", robot_frame});
  system_node->set_parameter({"odom_frame", odom_frame});
  system_node->set_parameter({"map_frame", map_frame});
  system_node->set_parameter({"world_frame", world_frame});

  // Build the expected TFInfo as SystemNode prepares it for subnodes.
  easynav::TFInfo expected_tf;
  expected_tf.tf_prefix = tf_prefix;
  expected_tf.robot_frame = robot_frame;
  expected_tf.odom_frame = odom_frame;
  expected_tf.map_frame = map_frame;
  expected_tf.world_frame = world_frame;

  // Trigger configuration so that SystemNode pushes parameters down to
  // subnodes (ControllerNode, LocalizerNode, MapsManagerNode, etc.). The
  // actual plugin loading will use these parameters to build TFInfo.
  auto state = rclcpp_lifecycle::State();
  auto ret = system_node->on_configure(state);
  ASSERT_EQ(ret, easynav::SystemNode::CallbackReturnT::SUCCESS);

  const easynav::TFInfo & actual_tf = easynav::RTTFBuffer::getInstance()->get_tf_info();

  EXPECT_EQ(actual_tf.tf_prefix, expected_tf.tf_prefix);
  EXPECT_EQ(actual_tf.robot_frame, expected_tf.robot_frame);
  EXPECT_EQ(actual_tf.odom_frame, expected_tf.odom_frame);
  EXPECT_EQ(actual_tf.map_frame, expected_tf.map_frame);
}


TEST_F(SystemTFInfoTest, SystemNodePropagatesFrameParametersWithNS)
{
  // Create a SystemNode as in production.
  auto system_node = std::make_shared<easynav::SystemNode>();

  // Set high-level frame parameters.
  const std::string map_frame = "my_map";
  const std::string odom_frame = "my_odom";
  const std::string robot_frame = "my_base";
  const std::string world_frame = "my_earth";
  const std::string tf_prefix = "robot1";

  // NOTE: we only set parameters on SystemNode; the ControllerNode will
  // declare and read them during its own on_configure() call triggered by
  // SystemNode::on_configure().
  system_node->set_parameter({"tf_prefix", tf_prefix});
  system_node->set_parameter({"robot_frame", robot_frame});
  system_node->set_parameter({"odom_frame", odom_frame});
  system_node->set_parameter({"map_frame", map_frame});
  system_node->set_parameter({"world_frame", world_frame});

  // Build the expected TFInfo as SystemNode prepares it for subnodes.
  easynav::TFInfo expected_tf;
  expected_tf.tf_prefix = tf_prefix;
  expected_tf.robot_frame = expected_tf.tf_prefix + "/" + robot_frame;
  expected_tf.odom_frame = expected_tf.tf_prefix + "/" + odom_frame;
  expected_tf.map_frame = expected_tf.tf_prefix + "/" + map_frame;
  expected_tf.world_frame = expected_tf.tf_prefix + "/" + world_frame;

  // Trigger configuration so that SystemNode pushes parameters down to
  // subnodes (ControllerNode, LocalizerNode, MapsManagerNode, etc.). The
  // actual plugin loading will use these parameters to build TFInfo.
  auto state = rclcpp_lifecycle::State();
  auto ret = system_node->on_configure(state);
  ASSERT_EQ(ret, easynav::SystemNode::CallbackReturnT::SUCCESS);

  const easynav::TFInfo & actual_tf = easynav::RTTFBuffer::getInstance()->get_tf_info();

  EXPECT_EQ(actual_tf.tf_prefix, expected_tf.tf_prefix);
  EXPECT_EQ(actual_tf.robot_frame, expected_tf.robot_frame);
  EXPECT_EQ(actual_tf.odom_frame, expected_tf.odom_frame);
  EXPECT_EQ(actual_tf.map_frame, expected_tf.map_frame);
}
