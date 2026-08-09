# V52 X36 convoy tracking finding

## Result

V52 was evaluated on `x36-formation-fov-convoy`, seed 1009, against the saved
paired V46 baseline. It preserved exactly one complete residual pulse per B4
window and therefore attempted the same 7,200 posterior messages as V46.

| Metric | V46 | V52 | Improvement |
|:--|--:|--:|--:|
| Full-horizon position E-OSPA | 126.370 | 128.241 | -1.48% |
| Focus-window position E-OSPA | 123.494 | 125.983 | -2.02% |
| Mean absolute cardinality error | 14.534 | 15.140 | -4.17% |
| Mean inter-formation position OSPA | 120.032 | 119.020 | +0.84% |
| Attempted messages | 7,200 | 7,200 | 0.00% |

All 39 post-bootstrap windows executed their pulse on phase 4. Across 117
eligible decisions, no serve action passed the retention gate. The current
pulse reduced one-round posterior disagreement by 4.15% on average and met
the disagreement-improvement condition on 98.29% of decisions, yet its mean
expected-cardinality change was -3.46% and its mean supported-label retention
risk was 16.86%. V52 therefore became a fixed phase-4 schedule rather than an
adaptive scheduler.

## Interpretation

The result rejects global timing of the complete residual layer. Delaying all
residual messages also delays local within-formation residual fusion, although
the V51 evidence implicated only a small number of cross-formation inputs.
The full phase-4 recovery then restores every harmful cross input together
and worsens cardinality and tracking. The 0.84% consensus improvement is not
a tracking benefit; it is another example of nodes agreeing more closely on
a worse estimate.

V52 is not tuned further and is not run on additional scenes. Its exact
serve/hold evaluator remains useful, but the action must become selective.
The next method keeps local residual timing unchanged and applies the actual
LMB counterfactual only to cross-formation residual inputs.

Development evidence only. The saved report and MAT artifact are in
`RUN/GA/dynamic_topology/evidence/formation_b4_v52_candidate_tracking_development/`.
