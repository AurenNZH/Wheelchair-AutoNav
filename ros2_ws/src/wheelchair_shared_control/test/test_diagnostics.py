import unittest

from diagnostic_msgs.msg import DiagnosticStatus

from wheelchair_shared_control.diagnostics import (
    SafetyDiagnosticSnapshot,
    build_safety_diagnostic_status,
    format_decision_transition,
    safety_diagnostic_values,
)
from wheelchair_shared_control.freshness import NAV2_LIVE
from wheelchair_shared_control.models import (
    CLEAR,
    SLOW,
    SafetyConfig,
    SafetyDecision,
)


class SafetyDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.config = SafetyConfig(
            slow_cost_threshold=5,
            stop_cost_threshold=90,
        )
        self.decision = SafetyDecision(
            decision=SLOW,
            permitted_forward=0.2,
            permitted_steering=0.0,
            reason="nav2_cost_slow",
            nearest_path_distance_m=0.8,
            maximum_path_cost=75,
            nearest_slow_cost_distance_m=0.8,
            nearest_stop_cost_distance_m=None,
            path_cost_valid=True,
        )

    def test_values_preserve_calibration_evidence_and_formatting(self):
        values = {
            item.key: item.value
            for item in safety_diagnostic_values(
                self.decision,
                self.config,
                12.5,
                1.25,
            )
        }

        self.assertEqual(values["maximum_path_cost"], "75")
        self.assertEqual(values["nearest_slow_cost_distance_m"], "0.800")
        self.assertEqual(values["nearest_stop_cost_distance_m"], "none")
        self.assertEqual(values["slow_cost_threshold"], "5")
        self.assertEqual(values["stop_cost_threshold"], "90")
        self.assertEqual(values["path_cost_valid"], "True")
        self.assertEqual(values["freshness_mode"], NAV2_LIVE)
        self.assertEqual(values["map_age_basis"], "receipt_time")
        self.assertEqual(values["source_age_ms"], "none")
        self.assertEqual(values["left_source_age_ms"], "none")
        self.assertEqual(values["turn_clearance_radius_m"], "0.450")
        self.assertEqual(values["clear_turn_limit"], "0.900")
        self.assertEqual(values["slow_turn_limit"], "0.600")
        self.assertEqual(values["turn_longitudinal_limit"], "0.150")
        self.assertEqual(values["reactive_assistance_mode"], "disabled")
        self.assertEqual(values["reactive_status"], "disabled")
        self.assertEqual(values["candidate_count"], "0")
        self.assertEqual(values["selected_steering"], "none")

    def test_status_metadata_and_levels_remain_stable(self):
        warning = build_safety_diagnostic_status(
            SafetyDiagnosticSnapshot(
                self.decision,
                self.config,
                map_age_ms=12.5,
                processing_ms=1.25,
            )
        )
        clear = build_safety_diagnostic_status(
            SafetyDiagnosticSnapshot(
                SafetyDecision(CLEAR, 0.5, 0.0, "nav2_cost_clear"),
                self.config,
                map_age_ms=12.5,
                processing_ms=1.25,
            )
        )

        self.assertEqual(warning.level, DiagnosticStatus.WARN)
        self.assertEqual(warning.message, "nav2_cost_slow")
        self.assertEqual(clear.level, DiagnosticStatus.OK)
        self.assertEqual(
            clear.name,
            "wheelchair_shared_control/safety_supervisor",
        )
        self.assertEqual(clear.hardware_id, "jetson")

    def test_transition_log_format_remains_stable(self):
        self.assertEqual(
            format_decision_transition(self.decision),
            "Safety decision changed: nav2_cost_slow "
            "(nearest=0.800 m max_cost=75)",
        )


if __name__ == "__main__":
    unittest.main()
