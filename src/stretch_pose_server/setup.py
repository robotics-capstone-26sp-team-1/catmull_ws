from setuptools import find_packages, setup

package_name = "stretch_pose_server"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kenneth Yang",
    maintainer_email="kjy5@uw.edu",
    description="ROS 2 package for stretch pose server.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "pose_server = stretch_pose_server.main:main",
            "find_board = stretch_pose_server.find_board:main",
        ],
    },
)
