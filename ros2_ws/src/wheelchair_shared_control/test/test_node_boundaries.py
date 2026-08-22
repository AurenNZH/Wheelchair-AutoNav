import ast
import unittest
from pathlib import Path


class SharedControlBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.package_dir = (
            Path(__file__).resolve().parents[1]
            / "wheelchair_shared_control"
        )

    def parse(self, module_name):
        return ast.parse((self.package_dir / module_name).read_text())

    def test_supervisor_has_no_top_level_policy_helpers(self):
        tree = self.parse("supervisor_node.py")
        functions = [
            node.name
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
        ]
        dataclasses = [
            node.name
            for node in tree.body
            if isinstance(node, ast.ClassDef)
            and any(
                isinstance(decorator, ast.Name)
                and decorator.id == "dataclass"
                for decorator in node.decorator_list
            )
        ]

        self.assertEqual(functions, ["main"])
        self.assertEqual(dataclasses, [])

    def test_executable_nodes_do_not_import_one_another(self):
        executable_modules = (
            "supervisor_node.py",
            "udp_bridge_node.py",
            "replay/envelope_monitor.py",
            "replay/intent_injector.py",
            "replay/map_restamper.py",
        )
        for module_name in executable_modules:
            with self.subTest(module=module_name):
                tree = self.parse(module_name)
                imported = {
                    node.module
                    for node in ast.walk(tree)
                    if isinstance(node, ast.ImportFrom) and node.module
                }
                self.assertFalse(
                    any(module.endswith("_node") for module in imported),
                    imported,
                )

    def test_pure_policy_modules_do_not_import_ros(self):
        pure_modules = (
            "freshness.py",
            "models.py",
            "operator_intent.py",
            "protocol.py",
            "safety_policy.py",
            "trajectory.py",
        )
        ros_prefixes = (
            "rclpy",
            "diagnostic_msgs",
            "geometry_msgs",
            "nav_msgs",
            "sensor_msgs",
            "std_msgs",
            "visualization_msgs",
            "wheelchair_msgs",
        )
        for module_name in pure_modules:
            with self.subTest(module=module_name):
                tree = self.parse(module_name)
                imported = {
                    node.module
                    for node in ast.walk(tree)
                    if isinstance(node, ast.ImportFrom) and node.module
                }
                imported.update(
                    alias.name
                    for node in ast.walk(tree)
                    if isinstance(node, ast.Import)
                    for alias in node.names
                )
                self.assertFalse(
                    any(
                        module.startswith(ros_prefixes)
                        for module in imported
                    ),
                    imported,
                )


if __name__ == "__main__":
    unittest.main()
