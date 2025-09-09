
import sys, time
from ros2cli.node.strategy import add_arguments, NodeStrategy
from ros2cli.verb import VerbExtension

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist

from ..controller.ros_controllers import TwistSubscriber, TwistStampedSubscriber


class EchoVerb(VerbExtension):
    """Print Twist in a single updating line for a given duration."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument('--duration', type=float, default=5000.0, help='Seconds to run')

    def print_twist(self, msg):
        line = (f"Twist  lin=({msg.linear.x:+.3f},{msg.linear.y:+.3f},{msg.linear.z:+.3f})  "
                f"ang=({msg.angular.x:+.3f},{msg.angular.y:+.3f},{msg.angular.z:+.3f})")
        sys.stdout.write('\r' + line + ' ' * 10); sys.stdout.flush()

    def twist_callback(self, msg):
        self.print_twist(msg)

    def twist_stamped_callback(self, msg):
        self.print_twist(msg.twist)

    def main(self, *, args):
        with NodeStrategy(args) as node:
            try:
                subs = {}
                subs['twist_subscriber'] = TwistSubscriber(node, self.twist_callback)
                subs['twist_stamped_subscriber'] = TwistStampedSubscriber(
                    node, self.twist_stamped_callback)

                self.twist_callback(Twist())

                t_end = time.time() + args.duration
                while time.time() < t_end:
                    rclpy.spin_once(node, timeout_sec=0.1)

            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            finally:
                sys.stdout.write('\n'); sys.stdout.flush()
            return 0
