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

- The initial `artifact_min_points_per_cell: 2` required two remaining accepted
  points.
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
blind-volume expansion. All point-count settings remain provisional and must
pass the recorded real-obstacle gates before the shadow can be considered for
any future safety use.

RViz also exposes the scope directly before further geometry changes are made.
Each mask has a bright green `XY HALO` footprint showing the continuous
sensor-frame comparison geometry. `/artifact_filter/threshold_cells` shows the
exact base-frame 10 cm cells evaluated on the current cloud: cyan for all
candidate cells and yellow for the subset cleared by insufficient support.
This diagnostic does not alter mask membership, thresholding, or either raw
map.

## Recorded residual geometry refinement — 2026-08-07

The external screenshot `artifact_mask_expanded_halo_filtering2.png` showed
persistent residuals near Mask 2, between Masks 2 and 0, and in sparse fans
beyond Masks 0 and 2. Exact bounds were derived from the external 95.4-second
bag `airy_artifact_geometry_20260806T222518Z`, containing 338 native
`/rslidar_points` clouds. Both files remain outside Git.

Masks 1–2 remain unchanged. Mask 0 was later extended 0.40 m from its outward
endpoint along its existing centreline, changing its end point from
`(0.1600, -0.5200)` to `(0.3247, -0.8845)` while retaining its width and Z
band. Eight sensor-frame prisms were also added:

```text
# mask purpose              start_x start_y end_x end_y half_width min_z max_z
3 gap                         0.205  -0.120  0.395 -0.120   0.030   0.040 0.140
4 Mask 2 corner               0.360  -0.170  0.490 -0.050   0.030   0.040 0.215
5 Mask 0 flare, left ray     -0.140  -0.480 -0.160 -0.880   0.030   0.035 0.150
6 Mask 0 flare, centre ray   -0.090  -0.400 -0.090 -0.990   0.035   0.035 0.150
7 Mask 0 flare, right ray    -0.040  -0.480  0.060 -1.090   0.040   0.035 0.150
8 Mask 2 negative diagonal    0.660  -0.160  0.960 -0.440   0.050   0.035 0.160
9 Mask 2 positive ray         0.730   0.000  1.050  0.080   0.050   0.075 0.160
10 Mask 2 horizontal ray      0.660  -0.180  0.980 -0.140   0.050   0.060 0.170
```

The three-ray Mask 0 representation cleared the recorded flare with about 30%
less unconditional XY prism area than one broad bounding prism. With all 11
masks, a 0.10 m halo, and a 10-point minimum, offline replay produced zero
occupied events in every manually labelled residual region across all 338
frames. Per-frame averages were 59.60 unique prism-rejected points, 24.80
threshold candidate cells, and 184.91 occupied shadow cells.

This empty-scene result changes only the diagnostic defaults; it is not an
obstacle-preservation result. The raw supervisor map remains authoritative,
and the panel, pole, block, and thin-horizontal-bar gates remain mandatory.

## Cell-aligned consolidation — 2026-08-07

A later room position produced a new low-height cloud near original Mask 1.
Adding another independent prism was rejected because the eleven-prism layout
did not express its relationship to the 10 cm front costmap clearly and had
become visually cluttered. The runtime geometry was replaced by three
`base_link` staircase regions:

```text
region 0: x5[-4..1], x6[-6..1], x7[-7..-6], x8[-8..-6],
          x9[-8..-6], x10[-9..-4], x11[-8..-4], x12[-8..-4]
region 1: x5[0..2], x6[0..4], x7[2..6], x8[2..4],
          x9[2..4], x10[2..3], x11[2..3]
region 2: x0..2[-6..-5]
```

The notation is `forward_cell[lateral_min..lateral_max]`. Region 2 copies the
original Mask 1 face one cell in robot-forward `+X`. There are 67 region-cell
records and 63 unique XY cells because regions 0 and 1 share four apex cells.
Each cell retains the Z band of the intersecting or nearest legacy prism rather
than receiving one broad region-wide height band.

The threshold halo is now an eight-neighbour, one-cell dilation of each exact
footprint. RViz shows three consolidated unlabeled meshes and their stepped
halo boundaries. The previous sections remain as the history of the legacy
prism calibration; those prisms are no longer runtime configuration. This
change remains shadow-only and does not alter the supervisor's raw map input.

Post-migration replay processed all 338 clouds from
`airy_artifact_geometry_20260806T222518Z`. The fixed mask-plus-halo scope
contained 139 unique cells. Average unique mask rejection was 73.65 points per
frame; average raw and shadow front occupancy was 209.70 and 184.97 cells.
All three staircase regions had zero residual occupied-cell events.

## Operator geometry and explicit halo — 2026-08-07

The operator subsequently refined the staircase masks to 76 cells across the
same three regions. Those edits are preserved as the current mask geometry.
Halo shape was then decoupled from the masks: `artifact_grid_halo_spans` now
stores inclusive lateral runs as
`[region_id, forward_cell, min_lateral_cell, max_lateral_cell]`.

The 23 spans reproduce a one-cell, eight-neighbour dilation of the
operator-refined masks. Future
halo tuning changes only these spans; every mask cell must remain covered, and
the mask cells and their per-cell Z bands remain unchanged.

Replay results for this operator-refined geometry are recorded after each
configuration update; the explicit halo must remain exactly reproducible from
the checked-in spans.

## Global shadow support and centred supervisor geometry — 2026-08-07

The two clear-scene bags exposed five intermittent occupied cells in the
straight swept path. Four were outside the local halo and survived on one or
two points. The diagnostic shadow now requires three post-filter points in
every front cell while preserving the stricter fifteen-point rule in the
configured mask/halo scope. Raw maps remain one-point-sensitive and unchanged.

The measured 0.80 m wheelchair base is centred on `base_link`; shared control
therefore uses 0.40 m forward and rear extents. STOP and SLOW travel-to-contact
distances remain 0.70 m and 1.20 m. A new clear-scene recording is required to
measure the combined result, with R1 retained as a labelled diagnostic because
it survived three frames of the previous second bag.
