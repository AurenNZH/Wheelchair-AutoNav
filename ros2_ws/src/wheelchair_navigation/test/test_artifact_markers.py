import unittest

import numpy as np
from std_msgs.msg import Header

from wheelchair_navigation.artifact_filter import (
    ArtifactGridCell,
    ArtifactHaloSpan,
)
from wheelchair_navigation.artifact_markers import (
    build_artifact_grid_markers,
    build_artifact_threshold_cell_markers,
)
from wheelchair_navigation.costmap import FrontCostmapConfig


class ArtifactMarkerTests(unittest.TestCase):
    def test_grid_markers_are_consolidated_unlabelled_and_cell_aligned(self):
        header = Header()
        header.frame_id = "base_link"
        config = FrontCostmapConfig(
            length_m=3.0, width_m=4.0, resolution_m=0.5
        )
        cells = (
            ArtifactGridCell(0, 1, 0, 0.2, 0.3),
            ArtifactGridCell(0, 2, 0, 0.25, 0.35),
        )
        halo_spans = tuple(
            ArtifactHaloSpan(0, forward, -1, 1)
            for forward in range(0, 4)
        )

        markers = build_artifact_grid_markers(
            header, cells, halo_spans, config
        ).markers

        self.assertEqual(len(markers), 4)
        self.assertEqual(markers[0].action, markers[0].DELETEALL)
        self.assertEqual(markers[1].ns, "artifact_grid_regions")
        self.assertEqual(markers[1].type, markers[1].TRIANGLE_LIST)
        self.assertEqual(markers[1].header.frame_id, "base_link")
        self.assertEqual(len(markers[1].points), 72)
        self.assertEqual(markers[2].ns, "artifact_grid_region_outlines")
        self.assertEqual(len(markers[2].points), 12)
        self.assertEqual(markers[3].ns, "artifact_grid_halo_outlines")
        self.assertFalse(
            any(marker.type == marker.TEXT_VIEW_FACING for marker in markers)
        )
        halo_xy = [(point.x, point.y) for point in markers[3].points]
        self.assertIn((0.0, -0.5), halo_xy)
        self.assertIn((2.0, 1.0), halo_xy)

    def test_threshold_cell_markers_match_front_grid_cell_centres(self):
        header = Header()
        header.frame_id = "base_link"
        config = FrontCostmapConfig(
            length_m=2.0, width_m=2.0, resolution_m=0.5
        )

        markers = build_artifact_threshold_cell_markers(
            header,
            config,
            candidate_cell_ids=np.array([0, 5, 15]),
            low_support_cell_ids=np.array([5]),
        ).markers

        self.assertEqual(len(markers), 2)
        self.assertEqual(markers[0].ns, "artifact_threshold_candidate_cells")
        self.assertEqual(markers[0].type, markers[0].CUBE_LIST)
        self.assertEqual(
            [(point.x, point.y) for point in markers[0].points],
            [(0.25, -0.75), (0.75, -0.25), (1.75, 0.75)],
        )
        self.assertEqual(
            [(point.x, point.y) for point in markers[1].points],
            [(0.75, -0.25)],
        )

        cleared = build_artifact_threshold_cell_markers(
            header,
            config,
            candidate_cell_ids=np.array([], dtype=np.int64),
            low_support_cell_ids=np.array([], dtype=np.int64),
        ).markers
        self.assertEqual(cleared[0].action, cleared[0].DELETE)
        self.assertEqual(cleared[1].action, cleared[1].DELETE)


if __name__ == "__main__":
    unittest.main()
