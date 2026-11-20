from setuptools import find_packages, setup

package_name = "xtrainer_mcp"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bringup.launch.py"]),
    ],
    install_requires=["setuptools", "mcp>=2,<3"],
    zip_safe=True,
    maintainer="kira",
    maintainer_email="kira@todo.todo",
    description="XTrainer MCP server for cameras, MoveIt, gripper and brakes",
    license="BSD-3-Clause",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "xtrainer_mcp = xtrainer_mcp.server:main",
        ],
    },
)
