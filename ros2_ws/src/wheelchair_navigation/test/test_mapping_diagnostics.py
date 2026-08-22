import unittest

from diagnostic_msgs.msg import DiagnosticStatus

from wheelchair_navigation.artifact_filter import (
    ArtifactCellSupportStats,
    ArtifactFilterStats,
)
from wheelchair_navigation.costmap import CostmapStats
from wheelchair_navigation.mapping_diagnostics import (
    MappingDiagnosticSnapshot,
    MappingMetrics,
    artifact_shadow_error_reason,
    build_mapping_diagnostic_status,
)


class MappingDiagnosticsTests(unittest.TestCase):
    def test_metrics_report_window_rate_and_latency(self):
        metrics = MappingMetrics(3)
        metrics.record(10.0, 1.0)
        metrics.record(20.0, 1.1)
        metrics.record(30.0, 1.2)

        values = metrics.values(lag_spike_ms=25.0)

        self.assertEqual(values["latency_window_count"], "3")
        self.assertEqual(values["processing_p50_ms"], "20.000")
        self.assertEqual(values["processing_p95_ms"], "29.000")
        self.assertEqual(values["processing_max_ms"], "30.000")
        self.assertEqual(values["mapping_p95_ms"], "29.000")
        self.assertEqual(values["cloud_age_p95_ms"], "0.000")
        self.assertEqual(values["lag_spike_count"], "1")
        self.assertEqual(values["effective_rate_hz"], "10.000")

    def test_status_formatter_preserves_names_messages_and_values(self):
        snapshot = MappingDiagnosticSnapshot(
            reason="ok",
            processing_ms=12.3456,
            cloud_age_ms=4.0,
            processing_warn_ms=100.0,
            cloud_age_warn_ms=100.0,
            processed_clouds=7,
            rejected_clouds=2,
            source_period_ms=100.0,
            arrival_period_ms=101.0,
            artifact_shadow_enabled=True,
            rolling_metrics={"effective_rate_hz": "9.900"},
            stage_ms={"decode_ms": 1.2345},
            stats=CostmapStats(10, 9, 8, 1, 7, 6, 5, 4),
            artifact_stats=ArtifactFilterStats(1, (3,), 3),
            artifact_support_stats=ArtifactCellSupportStats(
                2, 3, 4, 5, 6, 7, 8, 9, 10
            ),
            artifact_front_cells=11,
        )

        status = build_mapping_diagnostic_status(snapshot)
        values = {item.key: item.value for item in status.values}

        self.assertEqual(status.level, DiagnosticStatus.OK)
        self.assertEqual(status.message, "mapping_current")
        self.assertEqual(status.name, "wheelchair_navigation/local_costmap")
        self.assertEqual(status.hardware_id, "robosense_airy")
        self.assertEqual(values["processing_ms"], "12.346")
        self.assertEqual(values["decode_ms"], "1.234")
        self.assertEqual(values["raw_front_cells"], "4")
        self.assertEqual(values["artifact_shadow_enabled"], "true")
        self.assertEqual(values["artifact_region_0_rejected_points"], "3")
        self.assertEqual(values["artifact_low_support_points"], "10")

    def test_error_reason_precedes_latency_warning(self):
        common = dict(
            processing_ms=200.0,
            cloud_age_ms=200.0,
            processing_warn_ms=100.0,
            cloud_age_warn_ms=100.0,
            processed_clouds=0,
            rejected_clouds=1,
            source_period_ms=0.0,
            arrival_period_ms=0.0,
            artifact_shadow_enabled=False,
            rolling_metrics={},
        )
        warning = build_mapping_diagnostic_status(
            MappingDiagnosticSnapshot(reason="ok", **common)
        )
        error = build_mapping_diagnostic_status(
            MappingDiagnosticSnapshot(reason="missing_tf", **common)
        )

        self.assertEqual(warning.level, DiagnosticStatus.WARN)
        self.assertEqual(warning.message, "mapping_latency_exceeded")
        self.assertEqual(error.level, DiagnosticStatus.ERROR)
        self.assertEqual(error.message, "missing_tf")

    def test_artifact_shadow_errors_have_stable_reasons(self):
        self.assertEqual(
            artifact_shadow_error_reason(ValueError("frame mismatch")),
            "artifact_shadow_frame_mismatch",
        )
        self.assertEqual(
            artifact_shadow_error_reason(ValueError("bad cells")),
            "invalid_artifact_shadow_configuration",
        )


if __name__ == "__main__":
    unittest.main()
