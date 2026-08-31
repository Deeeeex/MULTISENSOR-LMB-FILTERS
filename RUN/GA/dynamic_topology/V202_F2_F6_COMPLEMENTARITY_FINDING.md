# V202 F2-to-F6 mode complementarity finding

## Result

Observation handover and precision refresh are recursively complementary under
the one-action-per-page deployable cap.  On the X36 `seed=211, t=72, H=3`
window, the teacher applies F2 source 19 label `[13,12]` at `t=72`, followed by
F6 source 19 label `[7,5]` at `t=73`.  Both use formation-level complete-label
multicast and residual label KLA with source weight `0.5`.

| Metric | Static full payload | V99 no repair | F2 then F6 |
|:--|--:|--:|--:|
| Mean E-OSPA | 85.970277 | 83.561598 | 82.577587 |
| Static-relative E-OSPA gain | 0 | +2.802% | +3.946% |
| Mean RMSE | 57.902417 | 58.288297 | 49.821562 |
| Static-relative RMSE gain | 0 | -0.666% | +13.956% |
| Consensus gain | 0 | +5.149% | +8.406% |
| Attempted-byte saving | 0 | +6.550% | +4.509% |

Relative to V99, the two-action sequence improves E-OSPA by `1.178%`, RMSE
by `14.526%`, and consensus by `3.434%`.  It adds `216,520 B` over V99 but
retains a fully charged positive communication saving relative to static.

## Complementarity

The paired single-action teachers on the same H=3 window produced:

| Teacher | Mean E-OSPA | Mean RMSE | Consensus gain | Byte saving |
|:--|--:|--:|--:|--:|
| F2 handover at t=72 | 82.834747 | 54.306048 | +7.406% | +5.211% |
| F6 precision refresh at t=72 | 82.893755 | 51.737171 | +7.433% | +5.754% |
| F2 at t=72, F6 at t=73 | 82.577587 | 49.821562 | +8.406% | +4.509% |

The delayed F6 action does not erase the F2 improvement.  Formation-2 and
formation-6 RMSE gains remain strongly positive at `63.564%` and `38.358%`,
and all six formations retain positive E-OSPA gains.  This closes the concern
that the two mode-specific residual updates merely compete for the same
posterior correction.

## Method consequence

The controller does not need to increase the deployable per-page action cap
to exploit both modes.  It needs a short causal action history and an explicit
abstention/value rule that can defer the second positive action to the next
page.  The mode classifier identifies the type of repair; a finite-horizon
value head and communication ledger decide its order and whether saved credit
still justifies execution.

The H=3 sequence does not address the later F3/F5 tails.  The next full-window
candidate should combine the proven F2-to-F6 prefix with the minimal F3
handover schedule after the ongoing no-t79 ablation decides whether the last
F3 transmission is redundant.

## Evidence boundary

The two source-label-page tuples are opened teacher routing keys and delivery
is ideal but charged.  This is one X36 H=3 mechanism result, not an online,
cross-seed, cross-scale, or cross-scene claim.
