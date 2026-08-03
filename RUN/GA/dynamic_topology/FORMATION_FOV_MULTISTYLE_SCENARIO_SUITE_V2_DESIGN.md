# Formation-FoV Multistyle Scenario Suite v2

## Why v1 cannot be used for tracking claims

The v1 implementation established three genuinely non-radial layouts, but
its five-seed geometry audit showed that the layouts were not yet comparable:

- M24 convoy had essentially zero multi-formation visibility and roughly
  `27%--29%` global blackout;
- crossing had roughly `35%--39%` global blackout;
- X36 relay had roughly `73%--76%` multi-formation visibility, substantially
  more overlap than M24 relay.

Those outcomes are scene-design failures, not evidence about the routing
method.  Opening tracking on them would confound network-scale transfer with
unmatched observability.

The same audit found two implementation risks.  Runtime overrides could
silently change the hardware or authorization flags of a named preset, and
numbered blockage pairs did not necessarily belong to the static reference
tree.  The reliable-MST constructor also selected edges using the complete
planned trajectory, which is acceptable for an offline illustration but is
not a causal online baseline.

## v2 contracts

All six M24/X36 presets remain selectable by one preset string, but a
versioned multistyle preset accepts **no runtime config overrides**.  A change
to hardware, geometry, topology construction or authorization must create a
new named scene version.

The immutable hardware contract remains:

- six sensors per formation;
- `120 deg` total FoV and `300 m` hard sensing range;
- detection probability `0.88` and measurement-noise standard deviation `7`;
- clutter rate `4` over the same registered global observation box;
- the complete `formation-shared-120deg-r300-q300-v1` quality profile;
- one shared boresight for every sensor in a formation.

`formalValidationAuthorized=false` and
`trackingOutcomeAuthorized=false` remain hard-coded while geometry
recalibration is in progress.  They cannot be changed through overrides.
The common event-triggered LMB filter enforces this flag, so a calibration
scene cannot reach tracking through a different screen or cache generator.
Flipping or deleting the flag after input generation also fails because the
complete scene contract is checked against a registered SHA-256 digest.  A
duplicated model-level gate envelope also preserves this rejection if the
nested scenario metadata is removed or its version marker is altered;
legacy models without any multistyle gate trace retain their historical
behavior.

The digest covers platform and target trajectories, sensing and clutter,
communication/drop parameters, edge budgets, topology construction, outage
times and authorization state.  A geometry or protocol change therefore
requires an intentional new registered contract rather than a silent edit.

## Causal reference topology and effective outages

The v2 reference tree is selected from physical geometry at `t=1` only.
Neither target truth, posterior state, future physical links nor future task
loss is read.  Budget filling is restricted to sensor edges within the
selected formation-level tree; it cannot silently add a non-tree
inter-formation shortcut.

Convoy and relay use an all-time-physical static tree.  Crossing intentionally
allows some static reference edges to become physically unavailable later and
is therefore treated as a stress scene.

Scene definitions register outage **times**, not assumed formation-pair IDs.
At input generation, those times are deterministically assigned to distinct
pairs of the actual t=1 reference tree.  Tests require each outage to affect
at least one physical reference edge during its window.

## Geometry changes

### Parallel convoy

M24 uses two lanes and X36 uses three lanes, with two longitudinal columns in
each scale.  Both scales use the same `200 m` local lane spacing and `350 m`
longitudinal column separation under the fixed `300 m` sensing range.  Scale
is increased by adding a lane, formations and traffic; it is not increased by
silently widening the local handoff gap.  Targets start just ahead of the
rear column, overtake the front column and make gentle parallel lateral
maneuvers.  Reused lane templates have fixed sub-lane offsets, avoiding the
near-collisions produced by v1's reciprocal lane changes.

### Linear relay

Formation spacing is fixed at `300 m` at both scales.  Target-route extent is
scaled with chain length, so the M24 and X36 cases expose comparable local
handover geometry rather than sending both target sets through one fixed
corridor length.

### Orthogonal crossing

The collision-safe v1 motion is retained.  Crossing remains a deliberately
hard development/stress family because its blackout is materially higher
than convoy and relay.  It must not enter the primary M24/X36 claim until a
separate frozen gate shows that observability has been repaired.

### Target kinematic safety

All v2 styles enforce target acceleration at or below `1 m/s^2` in addition
to the existing `14 m/s` speed cap.  Minimum target separation is also a hard
validation contract: `14 m` for convoy, `9 m` for relay, and `2 m` for the
deliberately close-passage crossing stress scene.  Impossible acceleration or
separation thresholds are covered by negative tests.

## Adversarial-review fixes

The first v2 review found that the new outage resolver accidentally rejected
legacy explicit schedules that block the same formation pair in two
non-overlapping windows.  Pair uniqueness is now restricted to the new
backbone-sequential mode, and the original D12 formation-FoV repeated-pair
schedule is a regression test.

The review also identified two tests that could pass on metadata alone.  The
causality test now holds all `t=1` positions fixed, arbitrarily perturbs every
future platform position and requires the reference tree to remain identical.
The outage test now requires every step of every registered window to contain
a physical reference edge and verifies an exact `0.65` probability increment
against a no-outage counterfactual schedule.

## Calibration plan and stopping rule

The primary non-radial matrix is:

1. radial formation-FoV control;
2. parallel convoy;
3. linear relay.

Each family must have paired M24 and X36 realizations.  Geometry calibration
uses no tracking filter and reports, for every seed:

- global, focus-window and worst-target blackout;
- maximum consecutive blackout;
- single- and multi-formation visibility;
- visible target load per sensor-time;
- focus-window handovers and formation-ownership entropy;
- static edge budget, t=1 tree identity and effective outage coverage.

Cross-scale gates compare M24 and X36 **within the same style**.  A main style
must pass every frozen seed before any tracking cache or outcome is opened.
Crossing is reported separately as stress-only.

After geometry thresholds and a source manifest are frozen, tracking may open
M24 development states first.  X36 is opened only if the same unchanged method
has positive M24 headroom under radial, convoy and relay.  No favorable style
may compensate for a failed style, and no geometry-calibration result is a
tracking or generalization result.
