# Matched-scale convoy geometry finding

## Evidence boundary

- Source commit: `d396a82` (`Harden multistyle formation-FoV scenarios`).
- Open development seeds: `[41, 43, 47, 53, 59]`.
- Presets: `m24-formation-fov-convoy` and
  `x36-formation-fov-convoy`.
- This is a sensing-geometry and communication-graph calibration result.
  It contains no tracking output and is not evidence of estimator or routing
  performance.

## Result

| Metric across five seeds | M24 | X36 |
|:--|--:|--:|
| Global blackout | 0.008--0.014 | 0.008--0.015 |
| Focus-window blackout | 0.012--0.021 | 0.012--0.021 |
| Worst-target blackout | 0.040--0.053 | 0.044--0.063 |
| Longest blackout | 6--8 steps | 6--8 steps |
| Single-formation visibility | 0.328--0.353 | 0.285--0.321 |
| Multi-formation visibility | 0.635--0.664 | 0.665--0.705 |
| Visible-target load per sensor-time | 4.750--4.771 | 5.142--5.156 |
| Focus-window ownership handovers | 16 | 24 |
| Minimum target separation | 16.007 m | 23.375 m |

All ten scenario realizations passed the hard geometry, topology, outage and
target-kinematic checks.  Mean global blackout is effectively unchanged
(`0.01115` for M24 and `0.01125` for X36).  X36 has modestly more overlapping
formation support and approximately `8.2%` greater per-sensor target load,
which is expected after adding one lane, two formations and eight targets.

## Research finding

The earlier X36 convoy deficit was not an unavoidable consequence of network
scale.  It was caused by changing the local sensing problem while scaling the
network: the formation columns and lanes were laid out differently relative
to the fixed `300 m` sensing range.  Once M24 and X36 share the same `200 m`
local lane spacing and `350 m` handoff distance, X36 no longer incurs a
systematic blackout penalty.  This supports a stricter experimental rule:
scale should be increased by adding formations and traffic while preserving
the local sensing and handoff geometry.

## Artifacts

The Markdown report is committed.  The reproducibility MAT file and raw log
remain available in the same local evidence directory but are intentionally
excluded by the experiment directory's `*.mat` / `*.log` ignore policy.

| Artifact | SHA-256 |
|:--|:--|
| `DYNAMIC_TOPOLOGY_SCENE_DIFFICULTY_20260803_115443.md` | `81c6ea132c96ca1b8b8858b6e579e67ea749cd65c16b7aba199890145b190681` |
| `DYNAMIC_TOPOLOGY_SCENE_DIFFICULTY_20260803_115443.mat` | `10f13bdc0d2ac5bd2e6b3aa0f5d530a175c90d456666ec7fa352b9b3409445f8` |
| `logs/MULTISTYLE_CONVOY_V2_R2_D396A82.log` | `447ca01ced5bc3c7efa9d05d05ad35aba00461cd9180a79436653d4323b6801d` |

The next gate must be committed before examining a fresh seed set.  Crossing
remains stress-only; convoy and relay are evaluated independently so that a
favorable style cannot compensate for a failed one.
