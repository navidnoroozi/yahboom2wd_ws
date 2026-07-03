from setuptools import setup
from glob import glob
import os

package_name = 'yahboom_2wd_tests'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@example.com',
    description='Odometry-feedback path-following test scenarios for Yahboom 2WD robots.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_follower_node = yahboom_2wd_tests.path_follower_node:main',
        ],
    },
)
