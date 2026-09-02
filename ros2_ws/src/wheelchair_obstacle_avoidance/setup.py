from setuptools import find_packages, setup


package_name = "wheelchair_obstacle_avoidance"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            "share/" + package_name + "/launch",
            ["launch/obstacle_avoidance.launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AurenNZH",
    maintainer_email="auren.ng@gmail.com",
    description="Reactive local-costmap steering-assistance launcher.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
