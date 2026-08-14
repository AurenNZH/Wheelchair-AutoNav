from glob import glob
from setuptools import find_packages, setup


package_name = "wheelchair_navigation"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AurenNZH",
    maintainer_email="auren.ng@gmail.com",
    description="Non-actuating LiDAR local obstacle mapping.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "artifact_point_filter = wheelchair_navigation.artifact_point_filter_node:main",
            "local_costmap = wheelchair_navigation.local_navigation_node:main",
            "mapping_monitor = wheelchair_navigation.mapping_monitor:main",
            "nav2_costmap_monitor = wheelchair_navigation.nav2_costmap_monitor:main",
        ],
    },
)
