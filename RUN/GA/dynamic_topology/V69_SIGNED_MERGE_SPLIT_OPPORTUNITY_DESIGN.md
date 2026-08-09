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
- Reference route: each receiver keeps the same 0.70 within-formation dominant
  input and one 0.05 residual input.  The residual route is induced by a
  deterministic path tree selected from the **current physical formation
  graph**.  Candidate paths maximize the weakest bidirectional sensor-pair
  support and then total support; selection reads no posterior, measurement
  support, link outcome, truth, or future geometry.
- No tracking score, future measurement, truth trajectory, or future outcome is
  available to the discovery code.

## Three signed quantities

For each current state, V69 measures three quantities on the same network
reference-existence denominator.

1. **Quarantine pressure.** Receiver-supported existence restored by
   withholding all residual inputs entering one formation for one round.  The
   frozen V65 1% risk gate and V66 0.05 robust decision-exposure gate jointly
   decide whether this is an actionable quarantine event.  The next two rounds
   return to the physical-tree reference and the complete sequence must pass
   the rolling-B3 connectivity reserve.
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

## Reference-feasibility correction

The first cache attempt exposed a baseline mismatch rather than a method
result.  The merge-split physical network remained connected throughout the
160-step scene, but a previously registered counter-clockwise cycle required
specific formation pairs that were no longer physical.  M24 has all 6 of 6
formation-pair links at every step; X36 has all 15 through step 135 and 14 of 15
thereafter.  Therefore widening communication range or weakening the scene
would hide the problem.  The corrected reference adapts only its path-tree
endpoints to the current physical action set and retains the same weights and
two-message-per-receiver budget.  It is feasible on all first 140 steps of both
frozen scenes; the weakest selected path still has all 36 possible sensor-pair
links on each selected formation edge.

## Decision rule

V69 is source discovery only.  It may identify candidate times but cannot run a
dynamic route or read tracking outcomes.  A subsequent paired experiment is
considered only when at least one scale contains a material, safe quarantine or
alternative-transport event.  The route implementation must then preserve the
fixed message budget and rolling connectivity and must be frozen before any
tracking outcome is opened.  If both scales fall below the source-side gates,
the merge-split scene is treated as insufficiently informative rather than
retuned after inspection.
