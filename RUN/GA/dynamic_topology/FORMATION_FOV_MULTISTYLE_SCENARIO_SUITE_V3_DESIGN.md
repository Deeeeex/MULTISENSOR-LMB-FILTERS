# Formation-FoV Multistyle Scenario Suite v3

## Why a new version is required

The complete v2 development audit at source `d396a82` passed all `30/30`
structural realizations, but it rejected relay as a main tracking scene.  Its
aggregate blackout was only `2.2%--3.4%`; nevertheless, one repeated
lowest-lane target suffered `28.7%--36.9%` blackout and up to `45`
consecutive blind steps.  The fail-closed assessment is archived at commit
`99f6abf`.

The v2 source contract states that a geometry change requires a new named
version.  v3 therefore preserves v2 evidence rather than rewriting it.
`formalValidationAuthorized=false` and
`trackingOutcomeAuthorized=false` remain unchanged: v3 is still geometry
calibration, not a tracking experiment.

## Executable diagnosis

For all five opened seeds, every relay blackout is caused by the hard
`300 m` range gate; no sample fails only because of the `120 deg` angular
gate.  The affected objects are local targets 3 and 4 of the lowest relay
lane, repeated as targets 19 and 20 in X36.  Their longest blind intervals
occur inside the route, not at birth, death or a route endpoint.

The v2 interaction was:

- stationary formation centres at `y=350 m` and `300 m` horizontal spacing;
- a lowest target lane at `y=80 m`;
- a `24 m` route bend;
- four targets per group with `28 m` cross-track spacing, hence a maximum
  `42 m` normal offset.

The lowest target consequently reaches `y=14 m`.  Its distance to the lower
part of two adjacent sensor rings leaves a gap between their `300 m` sensing
supports.  Formation radius/rotation jitter changes how the gap is split but
does not create it.

## Minimal repair

v3 changes one relay parameter:

```matlab
lanes = linspace(100, 220, min(4, targetGroupCount));
```

The upper lane, `24 m` bend, `28 m` within-group spacing, target counts,
sensor trajectories, communication graph, `120 deg / 300 m` hardware and
`300 m` formation spacing are unchanged.  On the worst opened seed (47),
this single change gives:

| Metric | M24 v2 | M24 v3 candidate | X36 v2 | X36 v3 candidate |
|:--|--:|--:|--:|--:|
| Global blackout | 2.79% | 0.57% | 3.39% | 0.77% |
| Worst-target blackout | 35.63% | 8.13% | 36.88% | 10.00% |
| Longest blind interval | 34 | 12 | 45 | 9 |
| Multi-formation visibility | 50.78% | 55.57% | 51.46% | 58.13% |
| Focus load per sensor-time | 4.917 | 5.248 | 4.576 | 5.017 |
| Focus handovers | 40 | 40 | 87 | 88 |
| Minimum target separation | 10.15 m | 12.34 m | 12.42 m | 14.77 m |

Lowering the sensor chain was rejected: `y=330 m` reduces minimum
sensor-target separation to about `13--17 m`, and `y=320 m` reduces it to
about `4--7 m`.  It repairs observability by creating a platform-target
near-collision.  The lane-only change leaves the observed minimum
sensor-target distance above `33 m`.

## New hard safety contract

v3 adds `minimumSensorTargetSeparation` to the scene SHA-256 contract and to
the common scenario validator.  Relay requires at least `30 m`; convoy and
crossing retain `0 m` because this new requirement was introduced to close
the diagnosed relay repair loophole, not to retrospectively tune their
geometry.  A negative regression requires an impossible cross-set distance
to fail closed.

The difficulty audit now reports both target-target and sensor-target
minimum separation.  This prevents a future calibration from improving FoV
statistics merely by moving sensors onto the target corridor.

Target state columns must also match the configured birth/death interval
exactly.  Every scheduled active column is fully finite and every scheduled
inactive column is fully NaN.  An all-NaN gap inside an active trajectory is
therefore rejected instead of silently removing a near-collision or blackout
sample from the safety and difficulty denominators.

## Calibration and stopping rule

The opened seeds `[41, 43, 47, 53, 59]` remain development-only.  First, v3
must pass the M24/X36 relay `2 x 5` audit while preserving target and
sensor-target safety.  Convoy and crossing receive new digests because the
suite contract version and safety field changed, but their geometry is not
re-tuned.

If the relay development audit succeeds, the absolute and cross-scale
difficulty gates are committed in a later scene version before any fresh
seed is opened.  Proposed relay limits are global blackout `<= 1%`,
worst-target blackout `<= 12%`, longest blackout `<= 14` steps, focus load
`4.0--5.5`, target-target separation `>= 9 m` and sensor-target separation
`>= 30 m`.  The held-out gate uses at least twenty new seeds and requires
every seed to pass.  A failure creates a new version and unopened seed set;
it does not permit relaxing the gate after inspection.

Crossing remains stress-only.  Convoy remains the only non-radial style that
has already passed the v2 development geometry gate.  No v3 geometry result
opens tracking or supports a routing-performance claim.
