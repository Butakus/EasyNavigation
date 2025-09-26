
from setuptools import setup
package_name = 'easynav_tools'
setup(
    name=package_name,
    version='0.1.1',
    packages=[package_name,
              # f'{package_name}.model',
              f'{package_name}.controller',
              f'{package_name}.tui',
              f'{package_name}.cli'],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools', 'rich>=13.3.0', 'pydantic>=2.0.0'],
    zip_safe=True,
    maintainer='Francisco Martín Rico',
    maintainer_email='fmrico@gmail.com',
    description='ROS 2 Navigation tools: TUI (Textual) + ros2cli commands for EasyNav.',
    license='GPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tui = easynav_tools.tui.app:run_app',
        ],
        # ros2 easynav <verb>
        'ros2cli.command': [
            'easynav = easynav_tools.cli.easynav:EasynavCommand',
        ],
        # IMPORTANT: extension point + verb group for ros2cli
        'ros2cli.extension_point': [
            'easynav.verb = ros2cli.verb:VerbExtension',
        ],
        'easynav.verb': [
            'navigation_control = easynav_tools.cli.navigation_control:NavigationControlVerb',
            'goal_info = easynav_tools.cli.goal_info:GoalInfoVerb',
            'twist = easynav_tools.cli.twist:TwistVerb',
            'nav_state = easynav_tools.cli.nav_state:NavStateVerb',
            'timestats = easynav_tools.cli.timetats:TimeStatsVerb',
            'plugins = easynav_tools.cli.plugins:PluginsVerb',
        ],
    },
)
