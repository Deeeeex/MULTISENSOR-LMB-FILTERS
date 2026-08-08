# V51 X36 convoy tracking finding

## Result

V51 was evaluated on `x36-formation-fov-convoy`, seed 1009, against the saved
paired V46 baseline. The scene, measurements, delivery draws and filter seed
were unchanged; only the V51 candidate was rerun.

| Metric | V46 | V51 | Improvement |
|:--|--:|--:|--:|
| Full-horizon position E-OSPA | 126.370 | 126.359 | +0.01% |
| Focus-window position E-OSPA | 123.494 | 122.983 | +0.41% |
| Mean absolute cardinality error | 14.534 | 14.314 | +1.51% |
| Mean inter-formation position OSPA | 120.032 | 118.120 | +1.59% |
| Attempted messages | 7,200 | 7,097 | 1.43% fewer |
| Attempted payload bytes | 233,938,560 | 226,698,352 | 3.10% fewer |

The controller deferred at least one formation on 34 of 40 pulse
opportunities (85%). It deferred 1.375 formations and 2.575 cross edges per
pulse on average. The action was therefore frequent enough to test the
mechanism; the near-zero full-horizon gain is not explained by a controller
that never acted.

## Decision

V51 is not advanced as the primary method. It demonstrates that withholding
some phase-1 cross-formation inputs can modestly protect cardinality and
reduce inter-formation disagreement, but removal-only control does not turn
that signal into a material tracking improvement. Its proxy threshold will
not be tuned against this opened X36 outcome.

V52 keeps the complete V46 pulse budget and changes only its service time.
Within each absolute B4 window (phases 1--4), the actual LMB serve/hold
counterfactual may delay the phase-1 pulse, re-evaluate it on phases 2 and 3,
and must restore one complete pulse by phase 4. This preserves `5N` attempted
messages per window while testing whether V51's small protection signal can
be combined with bounded information recovery.

Development evidence only. The saved report and MAT artifact are in
`RUN/GA/dynamic_topology/evidence/formation_b4_v51_candidate_tracking_development/`.
