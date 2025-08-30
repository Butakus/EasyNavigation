from setuptools import setup

package_name = 'easynav_support_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco Martín Rico',
    maintainer_email='fmrico@gmail.com',
    description='Python GoalManagerClient for EasyNav and integration tests against the C++ GoalManager.',
    license='GPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'demo_client = easynav_goalmanager_py.demo_client:main',
        ],
    },
)
