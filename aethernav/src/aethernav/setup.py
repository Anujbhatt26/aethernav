from setuptools import setup
import os
from glob import glob

package_name = 'aethernav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aethernav_team',
    maintainer_email='maintainer@example.com',
    description='Adaptive Multi-Agent Path Planning System',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent_node = aethernav.agent_node:main',
            'planner_node = aethernav.planner_node:main',
            'environment_node = aethernav.environment_node:main',
            'viz_node = aethernav.visualization.viz_node:main',
            'monitor_node = aethernav.monitoring.monitor_node:main',
        ],
    },
)