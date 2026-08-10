# V94 matched-static finding

## Decision

V94 is rejected as a cross-scale method. Under the corrected
`fov-aware-censored` receiver semantics, the persistent V65 rule retains
clear paired headroom at the two M24 development anchors but produces no
dynamic action at either registered X36 anchor. It therefore cannot support a
claim of stable M24/X36 improvement and is not authorized for full-episode
tracking.

## Matched comparison

Every row uses the same cached posterior, measurements, link uniforms, filter
RNG, three-step horizon and communication constraints. The reference is the
fixed counter-clockwise static route with full payload consumption.

| Scale | Anchor | Static E-OSPA | Candidate E-OSPA | Mean gain | Worst sensor | Minimum formation | Consensus | Attempted bytes | Gate |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| M24 | 104 | 71.664511 | 67.229679 | +6.188% | +18.852% | +0.000% | +13.641% | +1.501% | pass |
| M24 | 124 | 83.582917 | 77.559560 | +7.206% | +16.422% | +0.000% | +14.298% | +3.870% | pass |
| X36 | 72 | 85.970277 | 85.970277 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | fail |
| X36 | 100 | 89.375579 | 89.375579 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | fail |

The frozen gate required all four anchors to improve mean E-OSPA by at least
5% without regressions in worst-sensor, minimum-formation, consensus or
attempted-byte metrics. Only 2/4 anchors pass.

## Interpretation

The corrected semantics do not merely reduce the old X36 gain: they change
the observable risk calculation enough that the frozen 1% network-risk and
80% coverage rule falls back to the static route at both X36 anchors. The old
V65 X36 gains of +5.847% and +9.329% were therefore contingent on the former
`support-renormalized` receiver implementation and are not valid current
evidence.

This also closes the immediate baseline question. The method has meaningful
local M24 headroom relative to a matched static route, but no X36 improvement;
there is currently no validated cross-scale dynamic-routing strategy. A
stronger best-feasible-static comparison is unnecessary for V94 because it
already fails to improve over the canonical static reference on X36.

## Next method boundary

Do not retune the V65 risk threshold or coverage fraction on these opened
anchors. The next action family must affect a receiver population that grows
with network size rather than protecting a fixed local subset. The registered
fallback direction is a network-budget-reallocated multi-source handover
burst: reclaim low-value residual slots from non-critical rows, concentrate
several distinct cross-formation sources at the handover front, and optimize
the schedule over a horizon tied to graph diameter. Its first gate must again
be a matched static comparison, followed by a frozen best-feasible-static arm
before any full-episode claim.

This is opened development evidence only; it is not a held-out validation
result.
