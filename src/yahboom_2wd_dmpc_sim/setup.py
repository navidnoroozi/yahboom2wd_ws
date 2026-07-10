from setuptools import find_packages, setup

package_name = "yahboom_2wd_dmpc_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README_SIM.md"]),
        (f"share/{package_name}/launch", [
            "launch/two_robot_dmpc_sim.launch.py",
        ]),
        (f"share/{package_name}/config", [
            "config/two_robot_dmpc_sim.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Navid Noroozi",
    maintainer_email="navid@example.com",
    description="Simulation counterpart for two Yahboom 2WD distributed MPC experiments.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "two_robot_sim_node = yahboom_2wd_dmpc_sim.two_robot_sim_node:main",
            "analyze_two_robot_bag = yahboom_2wd_dmpc_sim.analyze_two_robot_bag:main",
        ],
    },
)
