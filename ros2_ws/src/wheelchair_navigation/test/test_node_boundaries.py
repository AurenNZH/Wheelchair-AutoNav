import ast
import unittest
from pathlib import Path


class NodeBoundaryTests(unittest.TestCase):
    def test_executable_nodes_do_not_import_one_another(self):
        package_dir = (
            Path(__file__).resolve().parents[1] / "wheelchair_navigation"
        )
        nodes = (package_dir / "point_support_filter_node.py",)

        for path in nodes:
            with self.subTest(node=path.name):
                tree = ast.parse(path.read_text())
                imported_modules = {
                    node.module
                    for node in ast.walk(tree)
                    if isinstance(node, ast.ImportFrom) and node.module
                }
                self.assertFalse(
                    any(
                        module.endswith("_node")
                        for module in imported_modules
                    ),
                    imported_modules,
                )


if __name__ == "__main__":
    unittest.main()
