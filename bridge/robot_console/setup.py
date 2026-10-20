from setuptools import setup

package_name = "robot_console"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/examples", ["examples/console.yaml"]),
        ("share/" + package_name + "/scripts", ["scripts/fetch_assets.sh", "scripts/mjcf_to_urdf.py"]),
    ],
    # The page is served off disk, so static/ ships beside the module. vendor/ is
    # fetched to ~/.local/share/robot_console, not shipped.
    package_data={package_name: ["static/*"]},
    install_requires=["setuptools", "pyyaml"],
    python_requires=">=3.8",
    zip_safe=False,
    maintainer="Lukas Mueller",
    maintainer_email="lukasbmueller@gmail.com",
    description="Web console for the robot bridge.",
    license="MIT",
    entry_points={"console_scripts": ["robot_console = robot_console.main:main"]},
)
