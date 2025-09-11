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

import re

_TAGS = {
    'black': '30', 'red': '31', 'green': '32', 'yellow': '33',
    'blue': '34', 'magenta': '35', 'cyan': '36', 'white': '37',
    'bold': '1', 'dim': '2', 'underline': '4'
}

_OPEN_TAG_RE = re.compile(r'\[([a-zA-Z]+)\]')
_CLOSE_TAG_RE = re.compile(r'\[/([a-zA-Z]+)\]')


def bbcode_to_ansi(s: str, enable_color: bool) -> str:
    if not enable_color:
        s = _OPEN_TAG_RE.sub('', s)
        s = _CLOSE_TAG_RE.sub('', s)
        return s

    for tag, code in _TAGS.items():
        s = re.sub(
            rf'\[{tag}\](.*?)\[/\s*{tag}\]',
            lambda m: f'\033[{code}m{m.group(1)}\033[0m',
            s,
            flags=re.DOTALL | re.IGNORECASE
        )
    s = _OPEN_TAG_RE.sub('', s)
    s = _CLOSE_TAG_RE.sub('', s)
    return s
