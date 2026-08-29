import unittest

from std_msgs.msg import Header
from visualization_msgs.msg import Marker

from wheelchair_navigation.artifact_filter import ArtifactBox, ArtifactHaloBounds
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

    def test_explicit_halo_geometry_overrides_box_margin(self):
        header = Header()
        header.frame_id = "base_link"
        box = ArtifactBox(0.1, 0.5, -0.3, 0.2, 0.4, 0.7)
        bounds = ArtifactHaloBounds(-0.3, 0.77, -0.7, 0.61)

        result = build_artifact_markers(
            header, box, 99.0, "lidar_right", bounds
        )

        halo = result.markers[2]
        coordinates = {(point.x, point.y) for point in halo.points}
        self.assertEqual(
            coordinates,
            {(-0.3, -0.7), (0.77, -0.7), (0.77, 0.61), (-0.3, 0.61)},
        )

    def test_additional_hard_box_is_a_distinct_cube(self):
        header = Header()
        header.frame_id = "base_link"
        box = ArtifactBox(-0.2, 0.67, -0.6, 0.14, 0.61, 0.77)
        joystick = ArtifactBox(0.5, 0.6, -0.3, -0.2, 0.77, 0.87)

        result = build_artifact_markers(
            header,
            box,
            0.1,
            "lidar_right",
            additional_boxes=(joystick,),
        )

        self.assertEqual(len(result.markers), 4)
        marker = result.markers[3]
        self.assertEqual(marker.type, Marker.CUBE)
        self.assertEqual(marker.ns, "lidar_right/additional_hard_box")
        self.assertAlmostEqual(marker.pose.position.x, 0.55)
        self.assertAlmostEqual(marker.pose.position.y, -0.25)
        self.assertAlmostEqual(marker.pose.position.z, 0.82)
        self.assertAlmostEqual(marker.scale.x, 0.10)
        self.assertAlmostEqual(marker.scale.y, 0.10)
        self.assertAlmostEqual(marker.scale.z, 0.10)
        self.assertTrue(marker.frame_locked)


if __name__ == "__main__":
    unittest.main()
