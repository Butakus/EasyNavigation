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

import sys
import time

from rclpy.executors import ExternalShutdownException

from ros2cli.node.strategy import add_arguments
from ros2cli.verb import VerbExtension

from ..controller.ros_controllers import LogReader


class TimeStatsVerb(VerbExtension):
    """Print Time stats info for a given duration."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        parser.add_argument('--duration', type=float, default=5000.0, help='Seconds to run')

    def main(self, *, args):
        try:
            log_reader = LogReader()

            t_end = time.time() + args.duration
            while time.time() < t_end:
                rows = log_reader.poll_time_stats_log()
                sys.stdout.write('\033[1;1H\033[J')
                sys.stdout.write('EasyNav Navigation Control Info:\n\n')
                sys.stdout.write(LogReader.rows2text(rows))
                sys.stdout.flush()
                time.sleep(0.5)

        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            sys.stdout.write('\n')
            sys.stdout.flush()
        return 0
