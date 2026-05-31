from setuptools import find_packages, setup
from glob import glob

package_name = "assistfour"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kenneth",
    maintainer_email="kjy5@uw.edu",
    description="TODO: Package description",
    license="apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "main = assistfour.main:main",
        ],
    },
)
