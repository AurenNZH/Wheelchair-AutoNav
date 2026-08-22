from glob import glob

from setuptools import find_packages, setup


package_name = "wheelchair_shared_control"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AurenNZH",
    maintainer_email="auren.ng@gmail.com",
    description="Fail-safe LiDAR shared-control supervisor.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "operator_intent_injector = "
            "wheelchair_shared_control.replay.intent_injector:main",
            "replay_map_restamper = "
            "wheelchair_shared_control.replay.map_restamper:main",
            "safety_supervisor = wheelchair_shared_control.supervisor_node:main",
            "safety_envelope_monitor = "
            "wheelchair_shared_control.replay.envelope_monitor:main",
            "udp_bridge = wheelchair_shared_control.udp_bridge_node:main",
        ],
    },
)
