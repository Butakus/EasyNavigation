# Copyright 2025 Intelligent Robotics Lab
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException

from ros2cli.node.strategy import add_arguments, NodeStrategy
from ros2cli.verb import VerbExtension

from ..cli.verb_utils import bbcode_to_ansi
from ..controller.ros_controllers import TwistProcessor, TwistStampedProcessor


class TwistVerb(VerbExtension):
    """Print Twist and TwistStamped for a given duration."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument('--duration', type=float, default=5000.0, help='Seconds to run')

    def twist_callback(self, msg):
        raw_text = TwistProcessor.msg2text(msg)

        enable_color = sys.stdout.isatty() and not os.environ.get('NO_COLOR')
        text = bbcode_to_ansi(raw_text, enable_color)

        sys.stdout.write('\033[1;1H\033[J')
        sys.stdout.write('EasyNav Twist:\n\n')
        sys.stdout.write(text)

        if enable_color:
            sys.stdout.write('\033[0m')
        sys.stdout.flush()

    def twist_stamped_callback(self, msg):
        raw_text = TwistStampedProcessor.msg2text(msg)

        enable_color = sys.stdout.isatty() and not os.environ.get('NO_COLOR')
        text = bbcode_to_ansi(raw_text, enable_color)

        sys.stdout.write('\033[1;1H\033[J')
        sys.stdout.write('EasyNav Twist:\n\n')
        sys.stdout.write(text)

        if enable_color:
            sys.stdout.write('\033[0m')
        sys.stdout.flush()

    def main(self, *, args):
        with NodeStrategy(args) as node:
            try:
                twist_sub = TwistProcessor(node, self.twist_callback)
                twist_stamped_sub = TwistStampedProcessor(node, self.twist_stamped_callback)
                twist_sub
                twist_stamped_sub

                t_end = time.time() + args.duration
                while time.time() < t_end:
                    rclpy.spin_once(node, timeout_sec=0.1)

            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            finally:
                sys.stdout.write('\n')
                sys.stdout.flush()
            return 0
