# Copyright 2025 Intelligent Robotics Lab
# GPL-3.0-or-later
from __future__ import annotations

import threading
from enum import Enum, auto
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

# These message types are provided by your workspace.
# They must exist for the integration to work.
from easynav_interfaces.msg import NavigationControl
from nav_msgs.msg import Goals  # NOTE: Your workspace must provide this message type

class ClientState(Enum):
    IDLE = auto()
    WAITING = auto()
    ACTIVE = auto()

class GoalManagerClient:
    """A Python client compatible with the C++ GoalManager.

    It exchanges NavigationControl messages on the `easynav_control` topic and publishes the
    commanded goal pose on `goal_pose` (PoseStamped), mirroring the C++ behaviour.

    The client filters inbound control messages so it only reacts to messages where
    `nav_current_user_id == self.id` (and ignores its own outbound messages).

    Attributes:
        node: rclpy Node used to create pubs/subs and timers.
        id: String identifier for this client. Defaults to f"{node.get_name()}_goal_manager_client".
        state: A light client-side state machine (IDLE/WAITING/ACTIVE).
        last_control: The last NavigationControl message that matched this client.
        last_feedback: The last FEEDBACK-type NavigationControl message that matched this client.
    """

    def __init__(
        self,
        node: Node,
        control_topic: str = 'easynav_control',
        goal_topic: str = 'goal_pose',
        client_id: Optional[str] = None,
        qos_depth: int = 100,
    ) -> None:
        self.node = node
        self.control_topic = control_topic
        self.goal_topic = goal_topic
        self.id = client_id or (self.node.get_name() + '_goal_manager_client')
        self.state = ClientState.IDLE

        self._lock = threading.Lock()
        self.last_control: Optional[NavigationControl] = None
        self.last_feedback: Optional[NavigationControl] = None

        qos = QoSProfile(depth=qos_depth)
        self._control_pub = self.node.create_publisher(NavigationControl, self.control_topic, qos)
        self._goal_pub = self.node.create_publisher(PoseStamped, self.goal_topic, qos)
        self._control_sub = self.node.create_subscription(
            NavigationControl, self.control_topic, self._on_control, qos
        )

    # ----------------------- Public API -----------------------

    def send_goal(self, goal: PoseStamped) -> None:
        """Send a single goal pose as a navigation REQUEST.

        The method also publishes the PoseStamped to `goal_pose` for parity with the C++ stack.
        """
        req = NavigationControl()
        # Header + sequence are typically filled by the server on responses; we set minimal fields.
        req.type = NavigationControl.REQUEST
        req.user_id = self.id
        goals = Goals()
        goals.header = goal.header
        goals.goals.append(goal)  # type: ignore[attr-defined]
        req.goals = goals

        with self._lock:
            self.state = ClientState.WAITING
            self.last_control = None

        # Publish the explicit goal pose as well (GoalManager also listens to this topic)
        self._goal_pub.publish(goal)
        # Publish the control request
        self._control_pub.publish(req)

    def cancel(self) -> None:
        """Request cancellation for the current navigation."""
        msg = NavigationControl()
        msg.type = NavigationControl.CANCEL
        msg.user_id = self.id
        self._control_pub.publish(msg)

    def reset(self) -> None:
        """Reset the client-side state machine and cached messages."""
        with self._lock:
            self.state = ClientState.IDLE
            self.last_control = None
            self.last_feedback = None

    # ----------------------- Helpers -----------------------

    def wait_for(
        self,
        predicate,
        timeout_sec: float = 2.0,
        spin: bool = True,
        spin_period_sec: float = 0.01,
    ) -> bool:
        """Utility: wait until predicate() returns True or a timeout is reached.

        If `spin` is True, rclpy.spin_once(node, timeout_sec=spin_period_sec) is used
        to service callbacks while waiting.
        """
        import time
        start = time.monotonic()
        while time.monotonic() - start < timeout_sec:
            if predicate():
                return True
            if spin:
                rclpy.spin_once(self.node, timeout_sec=spin_period_sec)
        return predicate()

    # ----------------------- Callbacks -----------------------

    def _on_control(self, msg: NavigationControl) -> None:
        # Ignore our own outbound messages
        if msg.user_id == self.id:
            return
        # Only accept messages whose 'nav_current_user_id' matches our client id, if present
        if getattr(msg, 'nav_current_user_id', None) not in (None, '', self.id):
            return

        with self._lock:
            self.last_control = msg
            if msg.type == NavigationControl.FEEDBACK:
                self.last_feedback = msg
                # Do not mutate state on feedback
            elif msg.type in (
                getattr(NavigationControl, 'ACCEPT', None),
            ):
                self.state = ClientState.ACTIVE
            elif msg.type in (
                getattr(NavigationControl, 'FINISHED', None),
                getattr(NavigationControl, 'FAILED', None),
                getattr(NavigationControl, 'CANCELLED', None),
                getattr(NavigationControl, 'REJECT', None),
                getattr(NavigationControl, 'ERROR', None),
            ):
                # Any terminal response => back to IDLE
                self.state = ClientState.IDLE

# -------- Optional demo CLI --------

def _make_demo_goal(node: Node) -> PoseStamped:
    import math
    from rclpy.time import Time as RclTime
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = node.get_clock().now().to_msg()
    goal.pose.position.x = 1.0
    goal.pose.position.y = 0.0
    goal.pose.orientation.w = 1.0
    return goal

def main(args=None):
    rclpy.init(args=args)
    node = Node('goalmanager_py_demo')
    client = GoalManagerClient(node)

    goal = _make_demo_goal(node)
    client.send_goal(goal)

    # Wait briefly for ACCEPT or REJECT
    client.wait_for(lambda: client.last_control is not None, timeout_sec=3.0)

    if client.last_control is not None:
        node.get_logger().info(f'Result type: {client.last_control.type} '
                               f'from user {client.last_control.user_id} '
                               f'msg: {getattr(client.last_control, "status_message", "")}')
    else:
        node.get_logger().warn('No response received')

    # Request cancel (useful if the system does not move during the demo)
    client.cancel()
    client.wait_for(lambda: client.last_control and client.last_control.type == NavigationControl.CANCELLED,
                    timeout_sec=3.0)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
