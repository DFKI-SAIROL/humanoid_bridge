from setuptools import setup

package_name = "g1_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "mujoco"],
    python_requires=">=3.10",
    zip_safe=True,
    maintainer="Lukas Mueller",
    maintainer_email="lukasbmueller@gmail.com",
    description="MuJoCo G1 that stands in for the robot on /lowstate and /lowcmd.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "g1_sim = g1_sim.node:main",
        ],
    },
)
