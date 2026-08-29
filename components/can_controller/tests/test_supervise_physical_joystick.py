import importlib.util
from pathlib import Path
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "supervise_physical_joystick.py"
)
SPEC = importlib.util.spec_from_file_location(
    "supervise_physical_joystick", SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class PhysicalJoystickArgumentsTests(unittest.TestCase):
    def _parse(self, *extra):
        return MODULE._arguments(
            [
                "--can-interface",
                "can1",
                "--gateway-interface",
                "can0",
                "--device-slot",
                "2",
                "--jetson-address",
                "192.0.2.10",
                *extra,
            ]
        )

    def test_obstacle_auto_resume_is_enabled_by_default(self):
        args = self._parse()
        self.assertTrue(args.auto_resume_obstacle_stops)

    def test_rollback_flag_requires_joystick_release(self):
        args = self._parse("--require-release-after-obstacle-stop")
        self.assertFalse(args.auto_resume_obstacle_stops)


if __name__ == "__main__":
    unittest.main()
