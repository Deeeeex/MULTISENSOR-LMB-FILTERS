# V55 X36 receiver-projection-only result

## Frozen paired result

The `x36-formation-fov-convoy`, seed 1009 projection-only arm reused the
saved V46 baseline, delivery trace, and filter seed.  It retained every V46
message and disabled label-payload selection.  The only estimator change was
the receiver-side existence projection.

| Metric | V46 | V55 projection-only | Improvement |
|:--|--:|--:|--:|
| Full-horizon position E-OSPA | 126.370 | 124.154 | +1.75% |
| Focus-window position E-OSPA | 123.494 | 120.718 | +2.25% |
| Worst-sensor position E-OSPA | 134.295 | 131.660 | +1.96% |
| Mean absolute cardinality error | 14.534 | 14.115 | +2.88% |
| Mean inter-formation position OSPA | 120.032 | 118.521 | +1.26% |
| Focus inter-formation position OSPA | 127.000 | 125.309 | +1.33% |
| Terminal inter-formation position OSPA | 110.635 | 115.710 | -4.59% |
| Attempted payload bytes | 233,938,560 | 236,006,448 | -0.88% |
| Delivered payload bytes | 227,975,304 | 230,029,392 | -0.90% |
| Filter runtime | 3,036.6 s | 4,541.9 s | -49.6% |

The arm kept the same 7,200 attempted and 7,026 delivered message
opportunities.  It detected 8,725 receiver-retention violations and resolved
all of them by existence clamps: there were no label removals, fallbacks, or
unresolved violations.

## Causal interpretation

This result establishes that the receiver projection is not merely a safety
repair.  On this X36 development pair it behaves as a robust-fusion
intervention and improves full-horizon tracking, focus tracking, worst-sensor
tracking, and cardinality.  The V54 failure therefore came largely from
combining projection with an unsuitable payload-selection objective and
context, not from projection alone.

The effect is nevertheless too small and too costly to satisfy the original
dynamic-topology target.  Full-horizon improvement remains below 2%,
attempted and delivered bytes increase, runtime increases by about one half,
and terminal inter-formation disagreement worsens.  One development pair is
also insufficient to claim a general robust-fusion benefit.

## Branch decision

The V55 causal attribution is now closed.

- Retain projection-only as an independent robust LMB-fusion ablation and a
  possible future side branch.
- Do not run the combined selection-plus-projection arm: selection-only has
  already worsened tracking and offers little direct saving.
- Do not train a GNN on V54/V55 posterior-preservation targets.  Those targets
  reproduce V46 fusion rather than predict downstream tracking improvement.
- Return the active method line to tracking-aligned dynamic topology.  V56
  must first demonstrate at least 5% oracle headroom at both M24 and X36
  before model training.

Development evidence only.  The paired machine report is
`evidence/formation_b4_v55_attribution_tracking_development/FORMATION_B4_V55_ATTRIBUTION_TRACKING_20260809_151421.md`.
