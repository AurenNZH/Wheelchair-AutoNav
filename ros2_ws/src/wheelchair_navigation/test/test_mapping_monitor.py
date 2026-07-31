import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from wheelchair_navigation.mapping_monitor import (
    MAPPING_STATUS_NAME,
    diagnostic_values,
    duplicate_fully_qualified_names,
    find_mapping_status,
    format_mapping_status,
)


class MappingMonitorTests(unittest.TestCase):
    def test_finds_mapper_status_and_formats_compact_line(self):
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = MAPPING_STATUS_NAME
        status.message = "mapping_current"
        status.values = [
            KeyValue(key="effective_rate_hz", value="10.0"),
            KeyValue(key="processing_ms", value="12.3"),
            KeyValue(key="cloud_age_ms", value="12.0"),
            KeyValue(key="occupied_cells", value="8"),
            KeyValue(key="front_occupied_cells", value="6"),
            KeyValue(key="self_filtered_points", value="3"),
            KeyValue(key="rejected_clouds", value="0"),
        ]
        msg = DiagnosticArray()
        other = DiagnosticStatus()
        other.name = "other"
        msg.status = [other, status]

        found = find_mapping_status(msg)
        line = format_mapping_status(found)

        self.assertIs(found, status)
        self.assertEqual(diagnostic_values(status)["processing_ms"], "12.3")
        self.assertIn("state=OK", line)
        self.assertIn("process=12.3ms", line)
        self.assertIn("cloud_age=12.0ms", line)
        self.assertIn("raw_cells=8 front_cells=6", line)
        self.assertIn("self_filtered_points=3", line)
        self.assertIn("rejected_clouds=0", line)

    def test_missing_mapper_status_returns_none(self):
        self.assertIsNone(find_mapping_status(DiagnosticArray()))

    def test_duplicate_node_names_include_namespaces(self):
        duplicates = duplicate_fully_qualified_names(
            [
                ("rslidar_sdk_node", "/"),
                ("rslidar_sdk_node", "/"),
                ("local_costmap", "/"),
                ("node", "/other"),
                ("node", "/other"),
            ]
        )

        self.assertEqual(
            duplicates, ["/other/node", "/rslidar_sdk_node"]
        )


if __name__ == "__main__":
    unittest.main()
