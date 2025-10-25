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

#include <string>
#include <atomic>
#include <thread>
#include <chrono>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_system/SystemNode.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::atomic_bool stop{false};

  std::thread rt_thread;
  {
    // Executors live in this scope and will be destroyed after join().
    rclcpp::executors::SingleThreadedExecutor exe_nort;
    rclcpp::executors::SingleThreadedExecutor exe_rt;

    auto system_node = easynav::SystemNode::make_shared();

    exe_nort.add_node(system_node->get_node_base_interface());
    exe_rt.add_callback_group(system_node->get_real_time_cbg(),
                              system_node->get_node_base_interface());

    // TF on steady/wall time, independent from /clock.
    auto tf_node = rclcpp::Node::make_shared("tf_node");
    tf_node->set_parameter(rclcpp::Parameter("use_sim_time", false));
    exe_rt.add_node(tf_node);

    auto tf_clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    auto tf_buffer = easynav::RTTFBuffer::getInstance(tf_clock);

    for (auto & node : system_node->get_system_nodes()) {
      exe_nort.add_node(node.second.node_ptr->get_node_base_interface());
      if (node.second.realtime_cbg != nullptr) {
        exe_rt.add_callback_group(node.second.realtime_cbg,
                                  node.second.node_ptr->get_node_base_interface());
      }
    }

    // Lifecycle: configure -> activate
    system_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    if (system_node->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_ERROR(system_node->get_logger(), "Unable to configure EasyNav");
      rclcpp::shutdown();
      return 1;
    }
    system_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    if (system_node->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_ERROR(system_node->get_logger(), "Unable to activate EasyNav");
      rclcpp::shutdown();
      return 1;
    }

    bool use_real_time = true;
    system_node->declare_parameter("use_real_time", use_real_time);
    system_node->get_parameter("use_real_time", use_real_time);

    // Cooperative shutdown on SIGINT
    rclcpp::on_shutdown([&](){
      stop.store(true, std::memory_order_relaxed);
      exe_rt.cancel();
      exe_nort.cancel();
    });

    // RT thread
    rt_thread = std::thread(
      [&, tf_node, tf_buffer, system_node, use_real_time]() {
        if (use_real_time) {
          RCLCPP_INFO(system_node->get_logger(), "Selected Real-Time");
          sched_param sch; sch.sched_priority = 80;
          if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
            RCLCPP_WARN(system_node->get_logger(),
              "Failed to set Real Time. Running with normal priority.");
          }
        } else {
          RCLCPP_INFO(system_node->get_logger(), "Selected NO Real-Time");
        }

        // No dedicated spin thread; TF uses exe_rt.
        tf2_ros::TransformListener tf_listener(*tf_buffer, tf_node, /*spin_thread=*/false);

        rclcpp::WallRate rate(100);
        while (!stop.load(std::memory_order_relaxed)) {
          if (system_node->get_current_state().id() ==
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
          {
            system_node->system_cycle_rt();
          }
          exe_rt.spin_some(std::chrono::milliseconds(1));
          rate.sleep();
        }
      });

    // Non-RT loop
    rclcpp::WallRate rate(100);
    while (!stop.load(std::memory_order_relaxed)) {

      if (system_node->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        system_node->system_cycle();
      }

      exe_nort.spin_some(std::chrono::milliseconds(1));
      rate.sleep();
    }

    // Ensure stop flag visible and cancel executors (idempotent)
    stop.store(true, std::memory_order_relaxed);
    exe_rt.cancel();
    exe_nort.cancel();

  

  // Wait the RT thread to finish before shutting down ROS.
  if (rt_thread.joinable()) {
    rt_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}
