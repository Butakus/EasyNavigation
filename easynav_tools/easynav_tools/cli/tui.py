
import sys, time, runpy
from ros2cli.node.strategy import add_arguments
from ros2cli.verb import VerbExtension
from ..services.state_store import StateStore
from ..adapters.ros.ros_adapter import ROSRunner
from ..utils.exec_profiler import ExecProfiler



class TUI(VerbExtension):
    """Run the TUI."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        
    def main(self, *, args):
        runpy.run_module('easynav_tools.adapters.tui.app', run_name='__main__')