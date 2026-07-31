from glob import glob
from setuptools import setup

package_name = "wheelchair_simulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/urdf", glob("urdf/*")),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "moving_dummy = wheelchair_simulation.moving_dummy:main",
            "safe_cmd_adapter = wheelchair_simulation.safe_cmd_adapter:main",
            "sim_operator_intent = "
            "wheelchair_simulation.sim_operator_intent:main",
            "sim_scenario_runner = "
            "wheelchair_simulation.sim_scenario_runner:main",
        ]
    },
)
