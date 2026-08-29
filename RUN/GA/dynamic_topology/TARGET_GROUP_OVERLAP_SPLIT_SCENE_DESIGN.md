# Target-group overlap/split scene

## Purpose

The existing radial scene mixes several effects: sensor motion, changing
physical routes, target handover and dense central visibility.  The current
formation merge/split scene also changes sensor geometry, so it cannot isolate
whether a communication method preserves label information when targets
become difficult to distinguish.

This development scene changes only target information flow.  The sensor
formations reuse the exact stationary linear-relay chain, the same six sensors
per formation, the same aligned headings, and the frozen 120-degree / 300 m
sensing envelope.  Two target cohorts begin in separated service bands,
occupy the same set of lanes near the middle of the episode, exchange bands,
and separate again.  Longitudinal group offsets prevent artificial target
collisions while keeping the two cohorts simultaneously visible during the
overlap.

The scene asks a concrete question: after two target populations share the
same observing formations, does selective communication overwrite a useful
label state, and can the network restore the correct state when the cohorts
separate again?

## One-call presets

| Scale | Preset | Sensors | Formations | Targets | Target groups |
|:--|:--|--:|--:|--:|--:|
| M24 | `m24-formation-fov-target-overlap` | 24 | 4 | 16 | 4 |
| X36 | `x36-formation-fov-target-overlap` | 36 | 6 | 24 | 6 |

Both presets are available through
`generateDynamicTopologyScenarioInputs(presetName, seed)` and accept the same
experiment scripts as the other formation-FoV scenes.

## Geometry contract

- Sensor formation centres are identical to the corresponding linear-relay
  preset and remain stationary for all 160 steps.
- Every formation faces downward with a shared 120-degree FoV and a 300 m hard
  range; detection, clutter, measurement noise and sensor quality are
  unchanged.
- All target groups live for the complete episode.  Cohort-centre vertical
  separation is at least 80 m at entry and exit, falls to at most 3 m around
  the overlap window, and reverses sign after the cohorts exchange bands.
- The physical sensor graph stays feasible throughout the episode.  The
  ordinary sequential link-blockage schedule remains active so the scene does
  not assume perfect communication.
- Minimum target and sensor--target separations remain hard validation fields;
  overlap means shared sensing support, not collocated targets.

## Evidence boundary

The focused seed-41 geometry test passes independently at M24 and X36.  It
checks the stationary sensor motion, identical relay-chain waypoints, target
cohort separation/overlap/reseparation, hard safety distances and physical
graph validity.  No tracker outcome has been opened.

The presets therefore remain `development-only`.  They are a secondary
mechanism/stress scene, not part of the primary claim until the method first
passes radial M24/X36 and transfers without retuning to the already qualified
parallel-convoy and linear-relay families.  A later formal geometry version
must use a new seed manifest before aggregate tracking is reported.
