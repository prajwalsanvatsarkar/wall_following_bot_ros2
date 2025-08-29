from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'wall_following_bot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            # follower_node must be an entry point in your package (def main())
            'follower_node = wall_following_bot.follower_node:main',
        ],
    },
)

