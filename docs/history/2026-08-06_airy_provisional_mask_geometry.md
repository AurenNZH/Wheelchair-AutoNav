# AIRY Provisional Mask Geometry — 2026-08-06

The diagnostic shadow initially shipped with an empty mask list because the
photographs did not establish XYZ bounds. A live `rslidar` cloud was then
inspected specifically to make the filter geometry visible and tunable in
RViz. Three repeatable low planar streaks were observed near the sensor:

```text
# start_x start_y end_x end_y half_width min_z max_z (metres)
  0.02   -0.21    0.16  -0.52   0.040      0.03  0.10
 -0.28    0.08   -0.48   0.14   0.035      0.03  0.10
  0.39   -0.07    0.56   0.02   0.035      0.03  0.10
```

These are deliberately short, narrow, provisional prisms. They exclude the
larger nearby structures visible in the cloud. They are suitable for visually
understanding and tuning the shadow filter, not for promotion to shared
control. The raw `/front_costmap` remains the supervisor input.

RViz now defaults to the diagnostic shadow view: both raw obstacle-map displays
are off and the artifact-filtered front map is prominent. Each configured prism
publishes a translucent fill, bright wireframe, and `MASK n` label on
`/artifact_filter/masks`. The raw `/front_costmap` topic is still published and
remains the supervisor input; only its RViz display is disabled by default.

## Mask-local support addendum

Live inspection confirmed that the prism membership test was working as
designed. On one representative cloud, the prisms rejected 91 points and the
front map fell from 193 to 187 occupied cells. Six prism-touched cells cleared;
six remained occupied because an unmasked point rasterized into the same 10 cm
cell. The surviving points were mostly just outside the prism XY boundary, with
two points just above one prism's Z maximum. A 20-frame check also showed the
strongest residual cells in 18–20 frames, so temporal persistence would not
classify them as transient noise.

The shadow therefore adds a deliberately local point-support rule after prism
exclusion:

- `artifact_min_points_per_cell: 2` requires two remaining accepted points.
- `artifact_threshold_halo_m: 0.10` limits that rule to front-map cells touched
  by a prism-rejected point or containing a remaining point in the prism XY
  footprint expanded by 10 cm. The halo ignores Z so it includes the sparse
  angled points immediately above a prism.
- Cells outside that scope retain the mapper's original one-point sensitivity.
- A value of `artifact_min_points_per_cell: 1` disables the support threshold
  while leaving prism exclusion active.

Prism-rejected points remain magenta. Points rejected only for insufficient
cell support are published separately in yellow. PCA, clustering, persistence,
a global minimum-point threshold, and wider prisms are deferred: the local
threshold directly addresses the observed rasterization effect with a smaller
blind-volume expansion. The default of two points is provisional and must pass
the recorded real-obstacle gates before the shadow can be considered for any
future safety use.
