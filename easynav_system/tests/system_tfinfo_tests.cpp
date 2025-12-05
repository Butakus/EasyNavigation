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

// This test instantiates a real SystemNode, sets the frame-related
// parameters on it, and relies on its on_configure() logic to propagate
// those parameters into the subnodes. We then verify indirectly that the
// controller plugin (DummyController) sees frames consistent with those
// parameters by inspecting the TFInfo that would be built from them.
TEST_F(SystemTFInfoTest, SystemNodePropagatesFrameParameters)
{
  // Create a SystemNode as in production.
  auto system_node = std::make_shared<easynav::SystemNode>();

  // Set high-level frame parameters.
  const std::string map_frame = "world_map";
  const std::string odom_frame = "world_odom";
  const std::string robot_frame = "world_base";
  const std::string tf_prefix = "robot";

  // NOTE: we only set parameters on SystemNode; the ControllerNode will
  // declare and read them during its own on_configure() call triggered by
  // SystemNode::on_configure().
  system_node->set_parameter({"tf_prefix", tf_prefix});
  system_node->set_parameter({"robot_frame", robot_frame});
  system_node->set_parameter({"odom_frame", odom_frame});
  system_node->set_parameter({"map_frame", map_frame});

  // Build the expected TFInfo as SystemNode prepares it for subnodes.
  easynav::TFInfo expected_tf;
  expected_tf.tf_prefix = tf_prefix + "/";
  expected_tf.robot_frame = expected_tf.tf_prefix + robot_frame;
  expected_tf.odom_frame = expected_tf.tf_prefix + odom_frame;
  expected_tf.map_frame = expected_tf.tf_prefix + map_frame;

  // Trigger configuration so that SystemNode pushes parameters down to
  // subnodes (ControllerNode, LocalizerNode, MapsManagerNode, etc.). The
  // actual plugin loading will use these parameters to build TFInfo.
  auto state = rclcpp_lifecycle::State();
  auto ret = system_node->on_configure(state);
  ASSERT_EQ(ret, easynav::SystemNode::CallbackReturnT::SUCCESS);

  // We cannot directly reach into the private controller plugin from here,
  // but we can verify that the parameters on the controller node match the
  // TFInfo we expect SystemNode to have constructed.
  auto system_nodes = system_node->get_system_nodes();
  ASSERT_TRUE(system_nodes.find("controller_node") != system_nodes.end());

  auto controller_node = system_nodes["controller_node"].node_ptr;
  ASSERT_NE(controller_node, nullptr);

  // The controller node should now have the frame parameters that will be
  // used to build TFInfo inside ControllerNode::on_configure().
  easynav::TFInfo actual_tf;
  controller_node->get_parameter("tf_prefix", actual_tf.tf_prefix);
  controller_node->get_parameter("robot_frame", actual_tf.robot_frame);
  controller_node->get_parameter("odom_frame", actual_tf.odom_frame);
  controller_node->get_parameter("map_frame", actual_tf.map_frame);

  EXPECT_EQ(actual_tf.tf_prefix, expected_tf.tf_prefix);
  EXPECT_EQ(actual_tf.robot_frame, expected_tf.robot_frame);
  EXPECT_EQ(actual_tf.odom_frame, expected_tf.odom_frame);
  EXPECT_EQ(actual_tf.map_frame, expected_tf.map_frame);
}
