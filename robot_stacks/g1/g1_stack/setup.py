from setuptools import find_packages, setup

package_name = "g1_stack"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests", "tests.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "pyzmq", "msgpack", "msgpack-numpy"],
    python_requires=">=3.8",
    zip_safe=True,
    maintainer="Lukas Mueller",
    maintainer_email="lukasbmueller@gmail.com",
    description="G1 deploy layer: joint state, observations, GEAR goal publishing.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "gear_controller = g1_stack.nodes.gear_controller:main",
            "joint_probe = g1_stack.nodes.joint_probe:main",
            "dex3_probe = g1_stack.nodes.dex3_probe:main",
            "fixed_goals = g1_stack.nodes.fixed_goals:main",
        ],
    },
)
