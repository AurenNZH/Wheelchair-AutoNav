import unittest

from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from wheelchair_navigation.artifact_filter import ArtifactBox
from wheelchair_navigation.artifact_markers import build_artifact_markers


class ArtifactMarkerTests(unittest.TestCase):
    def test_builds_static_hard_box_and_halo_outline(self):
        header = Header()
        header.frame_id = "base_link"
        box = ArtifactBox(0.1, 0.5, -0.3, 0.2, 0.4, 0.7)

        result = build_artifact_markers(header, box, 0.1, "lidar_right")

        self.assertEqual(len(result.markers), 3)
        self.assertEqual(result.markers[0].action, Marker.DELETEALL)
        hard_box = result.markers[1]
        halo = result.markers[2]
        self.assertEqual(hard_box.type, Marker.CUBE)
        self.assertAlmostEqual(hard_box.scale.x, 0.4)
        self.assertAlmostEqual(hard_box.scale.y, 0.5)
        self.assertAlmostEqual(hard_box.scale.z, 0.3)
        self.assertEqual(halo.type, Marker.LINE_LIST)
        self.assertEqual(len(halo.points), 8)
        self.assertTrue(hard_box.frame_locked)
        self.assertTrue(halo.frame_locked)


if __name__ == "__main__":
    unittest.main()
