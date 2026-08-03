# Multistyle v3 development geometry gate assessment

## Frozen evidence boundary

- Executable scene source: `8c4372e`.
- Open development seeds: `[41, 43, 47, 53, 59]`.
- Relay and convoy were audited independently at M24 and X36.
- All `20/20` generated realizations passed the v3 structural, kinematic,
  topology and target-activity validators.
- No tracking filter was run.  These results do not establish routing,
  estimator or cross-scale performance.

## Gate decision

| Style | Decision | Reason |
|:--|:--|:--|
| Linear relay | **GO for a later frozen geometry gate** | The lane-only repair removes the repeated range blind strip at both scales while preserving handovers, local load and safety separation. |
| Parallel convoy | **NO-GO** | Strong visibility statistics hide repeated planar platform--target near-collisions; minimum separation is only `0.44--1.92 m`. |
| Orthogonal crossing | **Stress-only** | v3 did not alter its v2 geometry or reopen its rejected main-scene status. |

The complete `radial + convoy + relay` tracking matrix therefore remains
closed.  Relay success cannot compensate for the convoy safety failure.

## Relay result

| Metric across five seeds | M24 | X36 | Proposed per-seed limit |
|:--|--:|--:|--:|
| Global blackout | `0.004--0.006` | `0.004--0.008` | `<= 0.010` |
| Focus blackout | `0.001` | `0.004--0.006` | `<= 0.015` |
| Worst-target blackout | `0.062--0.081` | `0.062--0.100` | `<= 0.120` |
| Longest blackout | `10--12` steps | `5--9` steps | `<= 14` steps |
| Multi-formation visibility | `0.549--0.568` | `0.581--0.605` | `>= 0.50` |
| Focus target load / sensor-time | `5.22--5.25` | `4.98--5.02` | `4.0--5.5` |
| Focus handovers | `40` | `88` | diagnostic, preserved |
| Target-target separation | `12.34 m` | `14.77 m` | `>= 9 m` |
| Sensor-target separation | `36.31--43.95 m` | `33.43--36.64 m` | `>= 30 m` |

The maximum same-seed M24/X36 differences are `0.002` global blackout,
`0.005` focus blackout, `0.019` worst-target blackout, `6` consecutive
steps, about `0.041` single-/multi-formation fraction and `0.007` ownership
entropy.  The X36/M24 focus-load ratio is approximately `0.954--0.958`.
These values pass the cross-scale limits proposed before the v3 audit.

## Why convoy is rejected despite good visibility

Convoy has only `0.8%--1.5%` aggregate blackout and retains substantial
multi-formation overlap, but its target streams occupy the same planar lanes
as the moving sensor rings.  Four targets span offsets
`[-48, -16, 16, 48] m`, while each six-sensor formation occupies a jittered
ring of roughly `31.5--38.5 m` radius.  During the overtake, target paths
therefore cross the sensor ring rather than merely passing the formation.

The failure recurs across scales and seeds:

- M24 minimum sensor-target separation: `0.44--1.92 m`;
- X36 minimum sensor-target separation: `0.44--0.99 m`;
- representative failures occur both near the initial rear formation
  (`t=4--5`) and during the front-formation overtake (`t=85--110`);
- the closest pairs cover multiple local sensor and target indices, so
  changing ring phase or one random seed would only move the collision.

The v3 convoy preset kept `minimumSensorTargetSeparation=0` because that
metric was introduced to close the relay repair loophole.  Consequently,
`All validations passed=1` in the raw convoy report means that the registered
v3 contract passed; it does **not** mean that the newly observed convoy
safety defect is acceptable.

## Next version boundary

A convoy repair must create a new named scene version and must preserve the
failed v3 evidence above.  The current two-dimensional development candidate
uses interleaved sensor/target service lanes, longitudinally staggered target
platoons and a common formation boresight toward the monitored corridor.  It
is exploratory until implemented, hashed, regression-tested and committed.

Before any unopened seed is evaluated, the later version must freeze:

1. an absolute sensor-target separation requirement of at least `30 m`;
2. target-target separation, blackout, overlap, load and handover gates;
3. paired M24/X36 scale-difference limits;
4. an exact unopened seed manifest and an all-seeds-pass rule.

Any held-out failure creates another scene version and seed set.  It does not
permit relaxing a threshold after inspection.  Tracking remains unauthorized
until both convoy and relay pass the frozen geometry validation.

## Artifacts

The Markdown reports are committed with this assessment.  Reproducibility
MAT files and raw logs remain in the same local directories under the
experiment ignore policy.

| Artifact | SHA-256 |
|:--|:--|
| `relay_calibration_r1_8c4372e/DYNAMIC_TOPOLOGY_SCENE_DIFFICULTY_20260803_125510.md` | `f7fd23e0e58596901041e7e259d96952f2a7af18b36c7e057f99b18627663153` |
| `relay_calibration_r1_8c4372e/DYNAMIC_TOPOLOGY_SCENE_DIFFICULTY_20260803_125510.mat` | `9200ccbbb973943b4a1f71285969177db266c4463381efaf4202abbb7f55c482` |
| `relay_calibration_r1_8c4372e/RELAY_CALIBRATION_2X5_8C4372E.log` | `07408c20038f5ced0824846820e66b9bf73ef1c251717ab3db90b4d4507eb163` |
| `convoy_safety_r1_8c4372e/DYNAMIC_TOPOLOGY_SCENE_DIFFICULTY_20260803_130052.md` | `4744d0c0951cc38d41d04bc28dc6b1c7c760ae4036dd0a929497035e28f3fde1` |
| `convoy_safety_r1_8c4372e/DYNAMIC_TOPOLOGY_SCENE_DIFFICULTY_20260803_130052.mat` | `8a9dea6d889e80af671dccbabb11c4117a30682ef8ebd265132be06f69c0c7b2` |
| `convoy_safety_r1_8c4372e/CONVOY_SAFETY_2X5_8C4372E.log` | `b984626d59b711a8b73140ccf0b6d84aa2c089527665f231d164a0482ca379b5` |
