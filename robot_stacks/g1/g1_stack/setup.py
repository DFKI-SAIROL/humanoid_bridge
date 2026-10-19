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
    # motor_probe serves its page off disk, so the static tree ships beside the
    # module. Globbed per depth, not "**": setuptools here is 59.6, which predates
    # recursive package_data globs. vendor/three/examples/jsm/loaders/ is 6 deep.
    package_data={
        "g1_stack.motor_probe": [
            "static/*",
            "static/*/*",
            "static/*/*/*",
            "static/*/*/*/*",
            "static/*/*/*/*/*",
            "static/*/*/*/*/*/*",
        ]
    },
    install_requires=["setuptools", "numpy", "pyzmq", "msgpack", "msgpack-numpy", "pyyaml"],
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
            "motor_probe = g1_stack.nodes.motor_probe:main",
        ],
    },
)
