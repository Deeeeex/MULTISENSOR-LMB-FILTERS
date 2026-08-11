# V96 matched-static finding

## Baseline contract

The primary baseline is the fixed counter-clockwise static carrier route with
full posterior payload.  The reference and V96 arms share the same cached
posterior state, measurements, delivery uniforms, filter random seed, carrier
graph, fusion weights, horizon and communication constraints.  V96 changes
only which receiver formations temporarily withhold complete cross-formation
posterior payload while retaining the control synopsis.

The one-step V96 arm is a duration ablation, not the primary baseline.  A
method is considered useful only when the persistent arm improves the matched
static full-payload arm by at least 5% at every frozen M24 and X36 development
anchor, without worsening the worst sensor, weakest formation, consensus or
attempted bytes.

## Matched result

Positive percentages mean that persistent V96 has lower E-OSPA than the
matched static baseline.

| Scale | Anchor | Static full payload | V96 one step | V96 persistent | Gain / static | Gain / one step | Gate |
|:--|--:|--:|--:|--:|--:|--:|:--:|
| M24 | 104 | 71.664511 | 69.842132 | 67.229679 | +6.188% | +3.741% | pass |
| M24 | 124 | 83.582917 | 79.834853 | 77.559560 | +7.206% | +2.850% | pass |
| X36 | 72 | 85.970277 | 85.442705 | 84.623645 | +1.566% | +0.959% | fail |
| X36 | 100 | 89.375579 | 87.173619 | 84.946782 | +4.955% | +2.554% | fail |

The matched-static gate passes at 2 of 4 anchors, so full-episode tracking and
validation claims remain unauthorized.  V96 establishes a repeatable M24
signal, but it does not yet establish a stable cross-scale benefit.

## What this does and does not show

The result is evidence for adaptive posterior-participation control on a
static carrier route.  It is not evidence that changing the physical routing
tree is useful.  The separate V91 full-episode comparison found that the
supposedly dynamic physical-tree reference executed only one unique route in
both rigid-translation scenes and was therefore functionally static.  Against
that matched static route, the V89 adaptive candidate changed full-episode
E-OSPA by -0.541% on M24 and +0.122% on X36, failing the strong gate.

V95 likewise showed that reallocating a residual fusion weight had negligible
authority: +0.000299% over static on M24 and +0.252401% on X36.  The current
research direction should therefore be described as dynamic control of the
effective posterior-participation graph, not as validated dynamic physical
routing.  The next iteration must close the X36 gap against the same matched
static full-payload baseline before richer-scene transfer is opened.
