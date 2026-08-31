# V202 F6 precision-refresh finding

## Result

The unresolved X36 F6 candidate is a real recursive repair opportunity, not a
failure of the current risk proxy.  On `x36-formation-fov`, seed `211`, opened
time `72`, and `H=3`, one formation-level residual label-KLA action sends
source 19's complete Bernoulli GM density for label `[7,5]` to F6 with source
weight `0.5`.

| Metric | Static full payload | V99 no repair | F6 precision refresh |
|:--|--:|--:|--:|
| Mean E-OSPA | 85.970277 | 83.561598 | 82.893755 |
| Static-relative E-OSPA gain | 0 | +2.802% | +3.579% |
| Mean RMSE | 57.902417 | 58.288297 | 51.737171 |
| Static-relative RMSE gain | 0 | -0.666% | +10.648% |
| Consensus gain | 0 | +5.149% | +7.433% |
| Attempted-byte saving | 0 | +6.550% | +5.754% |

Relative to V99, the action improves E-OSPA by `0.799%`, RMSE by `11.239%`,
and consensus by `2.408%`.  It adds `84,424 B` over V99 while retaining a
fully charged positive saving relative to static.  F6's formation-level RMSE
gain is `56.035%` and its E-OSPA gain is `4.625%`.

## Method consequence

The action is observably distinct from the verified F2 repair.  F6 has high
receiver-source covariance compatibility and an extreme bounded log
precision advantage; F2 has low compatibility and high handover pressure.
A mode transform built from the same truth-free candidate metadata assigns
F2 handover/precision evidence `0.1051 / 0.0040` and F6
`0.0145 / 0.2686`.  The classifier can therefore separate observation
handover from precision refresh without numeric formation or label features.

F6 alone does not pass the complete development gate because it intentionally
changes only F6; the pre-existing F1, F2, F3, and F5 RMSE tails remain.  The
next bounded experiment combines the F2 and F6 modes on consecutive pages,
respecting the one-action-per-page deployable cap.

## Evidence boundary

The source, label, time, and formation are opened teacher routing keys, and
delivery is ideal but charged.  This result proves mechanism headroom on one
X36 H=3 window only.  It is not an online selector, a cross-seed result, or a
paper-facing validation claim.
