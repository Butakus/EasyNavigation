# Copyright 2025 Intelligent Robotics Lab
# GPL-3.0-or-later
import os
import time
import unittest

import pytest

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from easynav_goalmanager_py import GoalManagerClient, ClientState

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions

# The executable name is guessed as 'system_main' in the 'easynav_system' package.
# If your executable has a different name, set the environment variable
# EASYNAV_SYSTEM_EXECUTABLE to override it.
SYS_PKG = os.environ.get('EASYNAV_SYSTEM_PACKAGE', 'easynav_system')
SYS_EXE = os.environ.get('EASYNAV_SYSTEM_EXECUTABLE', 'goalmanager_test_main')

def generate_test_description():
    system = launch_ros.actions.Node(
        package=SYS_PKG,
        executable=SYS_EXE,
        name='system_node_test',
        parameters=[
            {'allow_preempt_goal': True},   # ensure preemption path is testable
            {'position_tolerance': 0.01},
            {'angle_tolerance': 0.01},
        ],
        output='screen',
    )

    return (
        launch.LaunchDescription([
            system,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'system': system}
    )

class TestGoalManagerIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('goalmanager_py_integration_test')
        self.addCleanup(self.node.destroy_node)

    def _make_goal(self, x=1.0, y=0.0):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.orientation.w = 1.0
        return goal

    def test_request_and_cancel(self, system):
        client = GoalManagerClient(self.node)
        goal = self._make_goal()

        # Send a goal request
        client.send_goal(goal)
        ok = client.wait_for(lambda: client.last_control is not None, timeout_sec=5.0)
        self.assertTrue(ok, 'No response to REQUEST')
        self.assertIsNotNone(client.last_control)
        # We accept either ACCEPT or REJECT depending on server state, but not ERROR
        self.assertNotEqual(client.last_control.type, getattr(client.last_control, 'ERROR', -999))

        # Now cancel whatever is active
        client.cancel()
        ok = client.wait_for(lambda: client.last_control and client.last_control.type == client.last_control.CANCELLED,
                             timeout_sec=5.0)
        self.assertTrue(ok, 'Did not receive CANCELLED after cancel()')

    def test_preemption_two_clients(self, system):
        client1 = GoalManagerClient(self.node, client_id='client1_py')
        client2 = GoalManagerClient(self.node, client_id='client2_py')
        g1 = self._make_goal(1.0, 0.0)
        g2 = self._make_goal(2.0, 0.0)

        client1.send_goal(g1)
        ok = client1.wait_for(lambda: client1.state == ClientState.ACTIVE, timeout_sec=5.0)
        self.assertTrue(ok, 'Client1 did not get ACCEPT/ACTIVE')

        # Preempt with client2
        client2.send_goal(g2)
        # Client1 should ultimately see a CANCELLED due to preemption
        ok1 = client1.wait_for(lambda: client1.last_control and client1.last_control.type == client1.last_control.CANCELLED,
                               timeout_sec=5.0)
        # Client2 should be ACTIVE after its ACCEPT
        ok2 = client2.wait_for(lambda: client2.state == ClientState.ACTIVE, timeout_sec=5.0)

        self.assertTrue(ok1, 'Client1 did not get CANCELLED (preempted)')
        self.assertTrue(ok2, 'Client2 did not get ACTIVE')

# Required by launch_testing
@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info, system):
        # Ensure the system node did not crash
        launch_testing.asserts.assertExitCodes(proc_info)
