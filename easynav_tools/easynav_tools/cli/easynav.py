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

from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension


class EasynavCommand(CommandExtension):
    """Top-level 'ros2 easynav' command."""

    def __init__(self):
        super().__init__()

    def add_arguments(self, parser, cli_name):
        self._subparser = parser
        add_subparsers_on_demand(
            parser, cli_name, '_verb', 'easynav.verb', required=False)

    def main(self, *, parser, args):
        if not hasattr(args, '_verb'):
            self._subparser.print_help()
            return 0

        extension = getattr(args, '_verb')

        return extension.main(args=args)
