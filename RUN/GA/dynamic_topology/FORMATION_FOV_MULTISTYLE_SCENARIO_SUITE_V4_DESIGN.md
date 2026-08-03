# Formation-FoV Multistyle Scenario Suite v4

## Why v4 is required

The source-frozen v3 relay audit passed its development geometry gate, but
the later convoy safety audit exposed a separate defect.  Convoy visibility
looked favorable while target paths repeatedly crossed the planar rings of
moving sensors.  Across five opened seeds, minimum sensor-target separation
was only `0.44--1.92 m` for M24 and `0.44--0.99 m` for X36.

This is a causal scene error, not a random outlier.  Four targets span
cross-track offsets `[-48, -16, 16, 48] m`; each formation contains six
sensors on a jittered `31.5--38.5 m` ring.  When both occupy the same lane,
an overtake must intersect the ring.  Changing a ring phase or seed merely
moves the near-collision to another sensor.

v4 preserves the failed v3 evidence at commit `749e77f`.  It does not use an
unmotivated altitude difference: the repository describes generic mobile
surveillance platforms and does not establish an air-to-ground task.
`formalValidationAuthorized=false` and
`trackingOutcomeAuthorized=false` remain fail closed.

## Two-dimensional offset-corridor design

The repaired convoy represents sensor formations travelling on service
corridors next to the monitored target corridors.

| Local quantity | v4 value | Purpose |
|:--|--:|:--|
| Sensor-lane pitch | `220 m` | Preserve the same local geometry when a lane pair is added at X36. |
| Sensor-lane shift | `-55 m` | Centre the combined sensor/target layout. |
| Target-lane shift | `+55 m` | Give each paired sensor/target corridor a `110 m` offset. |
| Front/rear sensor-column spacing | `300 m` | Retain overlapping handoff support under the fixed `300 m` range. |
| Target cross-track spacing | `20 m` | Keep four targets separated without spanning the sensor ring. |
| Target route bend | `8 m` | Preserve a gentle overtake manoeuvre. |
| Same-lane route-template anchor offset | `80 m` longitudinally | Stagger repeated target cohorts without artificial sub-lanes. |
| Common FoV boresight | `+30 deg` | Point every formation consistently toward the monitored corridors. |

For `n` sensor lanes, the centres are

```text
sensor lanes = centeredValues(n, 220) - 55
target lanes = centeredValues(n, 220) + 55
```

M24 uses two lane pairs and X36 uses three.  Scaling therefore adds one
locally identical sensor/target corridor pair rather than changing the
sensing problem around every existing formation.

The `80 m` quantity is a raw route-template anchor offset, not a claim that
two generated cohort centres remain exactly `80 m` apart at every common
time.  The trajectory generator normalizes each route to its registered
birth/death duration.  On the opened v4 candidate, synchronized same-lane
centre spacing spans about `47.5--69.9 m` for M24 and `40.9--60.2 m` for
X36.  Runtime safety is therefore established from generated trajectories
by the target-target separation gate, not inferred from the waypoint offset.

## Analytic safety margin

The maximum sensor-ring radius is `35 * 1.10 = 38.5 m`.  A four-target
platoon with `20 m` spacing has maximum cross-track offset `30 m`, and the
route centre bends by at most `8 m`.  Because repeated cohorts are separated
longitudinally rather than laterally, the worst possible vertical approach
between paired corridors is bounded below by

```text
110 - 38.5 - 30 - 8 = 33.5 m.
```

The registered hard requirement is `minimumSensorTargetSeparation=30 m`.
The `3.5 m` analytic reserve is independent of the opened random seeds and
cannot be obtained by hiding an active target state: v3 already made the
birth/death activity mask exact and fail closed.

## Development evidence and stopping rule

The candidate was explored only on already-opened seeds.  On seeds `47` and
`53`, the final local geometry produced:

| Metric | M24 | X36 |
|:--|--:|--:|
| Global blackout | `0.0035--0.0052` | `0.0036` |
| Focus blackout | `0` | `0` |
| Worst-target blackout | `0.0125--0.0187` | `0.0187` |
| Longest blackout | `2--3` steps | `3` steps |
| Multi-formation visibility | `0.446--0.467` | `0.551--0.577` |
| Focus target load / sensor-time | `4.54--4.58` | `4.72--4.75` |
| Focus handovers | `21--23` | `34--36` |
| Target-target separation | `20.0 m` | `20.0 m` |
| Sensor-target separation | `38.85--44.02 m` | `35.98--40.50 m` |

These numbers select a v4 candidate; they are not validation.  After the
source, SHA registry and regressions are committed, the exact source commit
must pass a complete M24/X36 convoy-and-relay `4 x 5` development audit.
Convoy and relay are evaluated independently; favorable values cannot be
averaged across styles.

Only after that audit passes may a later version freeze absolute and paired
cross-scale thresholds plus an unopened seed manifest.  At least twenty new
seeds must then pass every gate.  A failure requires a new version and a new
unopened set, not a retrospective threshold change.  No v4 geometry result
authorizes tracking.
