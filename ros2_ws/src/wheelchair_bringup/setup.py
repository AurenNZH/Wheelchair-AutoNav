from glob import glob
from setuptools import find_packages, setup


package_name = "wheelchair_bringup"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AurenNZH",
    maintainer_email="auren.ng@gmail.com",
    description="Top-level sensor and navigation launch files for the wheelchair.",
    license="MIT",
    tests_require=["pytest"],
)
