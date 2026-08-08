# V50 X36 convoy tracking finding

## Result

V50 is neutral because it executes the V46 fallback in every B4 window, not
because a different posterior-aware route happens to produce a small effect.
The candidate uses the same X36 convoy seed-1009 scene, measurements,
physical-UID delivery draws and filter seed as the saved V46 baseline.

| Metric | V46 | V50 | V50 change |
|:--|--:|--:|--:|
| Full-horizon position E-OSPA | 126.370 | 126.370 | 0.00% |
| Focus-window position E-OSPA | 123.494 | 123.494 | 0.00% |
| Worst-sensor position E-OSPA | 134.30 | 134.30 | 0.00% |
| Mean absolute cardinality error | 14.534 | 14.534 | 0.00% |
| Mean inter-formation position OSPA | 120.03 | 120.03 | 0.00% |
| Attempted / delivered messages | 7200 / 7026 | 7200 / 7026 | 0 / 0 |
| Attempted / delivered payload bytes | 233.94 MB / 227.98 MB | 233.94 MB / 227.98 MB | 0.00% / 0.00% |
| Nonreference cycle selection | — | 0 / 40 | 0% |

The V50 candidate arm took 4,744 seconds, versus 3,037 seconds for the saved
V46 arm. Its repeated label-set scoring therefore adds about 56% runtime in
this case without changing one executed route.

## Method decision

Do not relax the posterior-value gate or tune the cycle-ranking coefficients
against this outcome. V49 already shows that forcing a structurally better
cycle can worsen tracking, while V50 shows that the current sender-replacement
score finds no safe positive alternative. Together they reject cycle choice as
the next primary lever on X36 convoy.

V51 therefore removes the V50 cycle-enumeration layer and starts from the V46
B4 route. It uses the current posterior only to identify a cross-formation
pulse whose sender lacks receiver-supported label-existence mass, defers that
input for at most one pulse, and projects the last/current pulse union back to
strong connectivity when necessary. The retained 2% existence-debt threshold
comes from the earlier M24 retention mechanism, not from fitting X36 outcomes.

This finding is method-development evidence from one X36 scene and seed. It
rejects V50 as implemented; it does not establish the performance of V51 or a
general impossibility result for learned message-value prediction.
