from glob import glob

import os

from setuptools import setup

package_name = 'wireless_watcher'

setup(
    name=package_name,
    version='1.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rkreinin',
    maintainer_email='rkreinin@clearpathrobotics.com',
    description='A Python-based node which publishes connection information about a linux wireless interface.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wireless_watcher = wireless_watcher.wireless_watcher:main'
        ],
    },
)
