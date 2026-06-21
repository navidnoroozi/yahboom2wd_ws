from setuptools import setup, find_packages

package_name = 'yahboom_2wd_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Navid Noroozi',
    maintainer_email='navid@example.com',
    description='ROS 2 driver node for Yahboom ROS Robot Control Board V3.0 on a custom 2WD robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yahboom_2wd_node = yahboom_2wd_driver.yahboom_2wd_node:main',
            'yahboom_serial_probe = yahboom_2wd_driver.serial_probe:main',
            'yahboom_motor_test = yahboom_2wd_driver.motor_test:main',
        ],
    },
)
