# V55 X36 full-reference selection-only result

## Frozen paired result

The `x36-formation-fov-convoy`, seed 1009 selection-only arm reused the saved
V46 baseline, delivery trace, and filter seed.  It included every present V46
label object in the context-aware rate-distortion reference and disabled the
receiver existence projection.

| Metric | V46 | V55 selection-only | Improvement |
|:--|--:|--:|--:|
| Full-horizon position E-OSPA | 126.370 | 128.103 | -1.37% |
| Focus-window position E-OSPA | 123.494 | 125.306 | -1.47% |
| Worst-sensor position E-OSPA | 134.295 | 134.912 | -0.46% |
| Mean absolute cardinality error | 14.534 | 14.954 | -2.89% |
| Mean inter-formation position OSPA | 120.032 | 120.787 | -0.63% |
| Attempted payload bytes | 233,938,560 | 225,082,568 | +3.79% |
| Candidate filter runtime | 3,036.6 s | 4,103.6 s | -35.1% |

The arm kept all 7,200 attempted message opportunities and the same 7,026
delivered messages.  On 400 selectable edge-times, the candidate-state full
payload ledger contained 12,659,360 bytes.  The compact synopses and selected
GM labels used 553,344 and 11,845,984 bytes, respectively, giving only 2.05%
direct saving on the selective path.  No existence projection, removal,
fallback, or unresolved violation occurred.

## What this result isolates

Removing the V54 existence projection reduced the failure magnitude but did
not restore tracking.  V54 had changed full-horizon E-OSPA and cardinality by
-1.91% and -6.22%; V55 selection-only changed them by -1.37% and -2.89%.
Because the teachers also differ, this comparison is not a clean numerical
estimate of projection effect.  It is sufficient to reject the claim that
projection was the only reason V54 failed.

The full-reference correction also removed V54's evidence-gated deletion of
predicted tracks.  Tracking still deteriorated across full horizon, focus
window, cardinality, worst sensor, and inter-formation error.  Therefore the
context mismatch and positive-evidence reference bias were real defects, but
fixing them did not create useful X36 headroom.

## Fundamental objective mismatch

The V55 selection objective minimizes divergence from full-message V46
fusion under a lower byte budget.  This is a coherent compression objective:
it asks how cheaply the receiver can reproduce a chosen estimator.  It is not
a tracking-improvement objective.  Any omitted input moves the candidate away
from the V46 reference, and the teacher has no reason to prefer an omission
that improves E-OSPA or cardinality.

Consequently, this teacher can support a communication--accuracy trade-off or
a posterior-preservation claim, but it cannot naturally support the original
goal of clear tracking improvement over V46.  Such an improvement would
require full V46 fusion to be harmful and the policy to recognize harmful
inputs using a tracking-aligned causal criterion; reference-to-candidate KLD
penalizes exactly that departure.

The small 2.05% direct selective-path saving and 35.1% runtime increase also
make the current exact teacher unattractive as a practical endpoint.  The
larger 3.79% total byte difference includes downstream changes in posterior
complexity on later messages and must not be presented as the direct subset
optimizer saving.

## Next decision

Do not run the combined V55 arm and do not train a GNN on selection targets.
One clean causal question remains: whether the receiver existence projection
alone improves tracking when every V46 payload is retained.  Run the frozen
projection-only X36 arm next.

- If projection-only produces clear tracking and cardinality gains, recast the
  contribution as robust LMB fusion rather than communication routing.
- If projection-only is neutral or harmful, stop the label-payload branch and
  return to tracking-aligned dynamic topology design.  A later data-driven
  policy must predict downstream tracking value or regret while an explicit
  structural projection preserves the V46 information path.

Development evidence only.  The paired report is
`evidence/formation_b4_v55_attribution_tracking_development/FORMATION_B4_V55_ATTRIBUTION_TRACKING_20260809_134750.md`.
