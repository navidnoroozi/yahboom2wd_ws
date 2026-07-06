from setuptools import find_packages, setup

package_name = "yahboom_2wd_dmpc"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [
            "launch/robot_dmpc_controller.launch.py",
            "launch/two_robot_dmpc_coordinator.launch.py",
        ]),
        (f"share/{package_name}/config", [
            "config/two_robot_dmpc.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Navid Noroozi",
    maintainer_email="navid@example.com",
    description="ROS 2 and ZeroMQ interface for two Yahboom 2WD distributed MPC experiments.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dmpc_controller_node = yahboom_2wd_dmpc.dmpc_controller_node:main",
            "dmpc_coordinator_ros_node = yahboom_2wd_dmpc.dmpc_coordinator_ros_node:main",
        ],
    },
)
