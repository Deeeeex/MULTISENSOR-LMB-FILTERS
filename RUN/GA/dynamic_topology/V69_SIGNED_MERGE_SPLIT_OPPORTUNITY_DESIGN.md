# V69 signed merge-split opportunity design

## Question

Can the same current-posterior evidence that separates strong from weak
quarantine events also expose a materially useful transport action in a
non-radial scene, at both M24 and X36 scale?

The merge-split scene is chosen because its physical mechanism changes over
time.  Parallel branches first converge into a dense corridor, where repeated
or incompatible cross-formation information may be worth quarantining.  The
branches later separate, where recently observed information may instead need
to be retained or transported to a different receiver.  This gives both action
signs a causal reason to occur in one scene, rather than manufacturing a route
change in a relay whose fixed reference is already near the source-only upper
bound.

## Frozen scene boundary

- Presets: `m24-formation-fov-merge-split` and
  `x36-formation-fov-merge-split`.
- Development seed: `1401` for both scales.
- Source snapshots: `40:4:140`.
- Sensor hardware: common 120-degree FoV and 300 m range.
- All target streams start at time 1 and remain phase-aligned with the moving
  branches.  Their paths maintain at least 30 m sensor-target separation, so a
  useful event cannot be caused by a target spawning inside a formation.
- Reference route: the fixed counter-clockwise formation route already used by
  the tracking-aligned source-cache pipeline.
- No tracking score, future measurement, truth trajectory, or future outcome is
  available to the discovery code.

## Three signed quantities

For each current state, V69 measures three quantities on the same network
reference-existence denominator.

1. **Quarantine pressure.** Receiver-supported existence restored by
   withholding an existing cross-formation input.  The frozen V65 1% risk gate
   and V66 0.05 robust decision-exposure gate jointly decide whether this is an
   actionable quarantine event.
2. **Reference-edge retention pressure.** Sender-supported existence lost by
   withholding the same input.  This identifies the sign of the information
   flow but is not itself a new route.
3. **Alternative-sender headroom.** For each 0.05 residual input slot, replace
   its sender by the best currently physical but unused cross-formation sender,
   preserving receiver, message count, weight, and every other source.  A
   candidate must cause no supported downward 0.5 crossing, and its optimistic
   net headroom must reach 1% before transport projection is considered.

The third quantity is deliberately optimistic because connectivity has not yet
been projected.  If even this upper bound is below 1%, the state cannot justify
a route implementation at the current action strength.

## Decision rule

V69 is source discovery only.  It may identify candidate times but cannot run a
dynamic route or read tracking outcomes.  A subsequent paired experiment is
considered only when at least one scale contains a material, safe quarantine or
alternative-transport event.  The route implementation must then preserve the
fixed message budget and rolling connectivity and must be frozen before any
tracking outcome is opened.  If both scales fall below the source-side gates,
the merge-split scene is treated as insufficiently informative rather than
retuned after inspection.
