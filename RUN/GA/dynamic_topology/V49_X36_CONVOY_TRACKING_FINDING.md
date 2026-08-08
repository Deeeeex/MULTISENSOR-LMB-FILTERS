# V49 X36 convoy tracking finding

## Paired development result

The first full tracking comparison uses `x36-formation-fov-convoy`, seed
`1009`.  V46 and V49 share the generated scene, measurements, physical-UID
delivery draws and filter seed.  The scene contains 36 sensors in six
formations and 24 targets.  Positive improvement means the candidate is
better.

| Metric | V46 | V49 | V49 improvement |
|:--|--:|--:|--:|
| Full-horizon position E-OSPA | 126.370 | 128.626 | -1.79% |
| Focus-window position E-OSPA | 123.494 | 126.524 | -2.45% |
| Worst-sensor position E-OSPA | 134.300 | 134.650 | -0.26% |
| Mean absolute cardinality error | 14.534 | 15.335 | -5.51% |
| Mean inter-formation position OSPA | 120.030 | 126.430 | -5.33% |
| Focus inter-formation position OSPA | 127.000 | 134.090 | -5.58% |

V49 selected a feasible cycle in all `40/40` B4 windows.  Its mean registered
structural contraction gain was `3.6449%`, and the fraction of delivered B4
windows whose union was strongly connected increased from `0.45` to `0.90`.
Nevertheless, every main tracking or estimator-consensus metric moved in the
wrong direction.  This directly falsifies the working assumption that better
graph contraction and delivered connectivity are sufficient routing objectives
for this tracker.

## Communication interpretation

Both arms attempted `7200` posterior messages.  V46/V49 delivered
`7026/7027`, so message counts are effectively identical.  Attempted payload
grew from `233.94 MB` to `237.22 MB` (about `+1.40%`), and delivered payload
grew from `227.98 MB` to `231.58 MB` (about `+1.58%`).  The route changes the
posterior evolution and therefore message sizes even when scheduled edge counts
are equal.  V49 supplies neither a tracking gain nor a byte-saving result.

## Method decision

V49 is rejected as the next method direction.  This single development pair is
not a universal statistical claim about every cycle route, but it is sufficient
to reject the graph-only objective: its own intended structural quantities
improved strongly while tracking, cardinality and estimator agreement all
regressed.  Running more V49 seeds or tuning the 1% structural threshold would
not repair that objective mismatch.

The active next test is V50, which retains the feasible-cycle action space,
fixed B4 budget, exact structural no-worse gate and V46 fallback, while ranking
residual messages by current-posterior information transfer and formation-tail
utility.  The V50 candidate reuses this run's V46 summary and random seeds; only
the new arm is executed.  A useful V50 result must first reverse the cardinality
and E-OSPA regressions, not merely keep the graph connected.

Source result:
`RUN/GA/dynamic_topology/evidence/formation_b4_v49_paired_tracking_development/FORMATION_B4_V49_PAIRED_TRACKING_20260809_043212.{mat,md}`.
