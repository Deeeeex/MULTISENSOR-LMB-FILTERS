# V97 positive-net addressable payload finding

## Decision

V97 improves the matched static full-payload baseline at every opened anchor
and passes three of four strict gates.  It is a material improvement over V96,
but the cross-scale gate remains closed because X36 t=72 reaches only 2.412%
mean E-OSPA gain rather than the required 5%.

## Matched result

All arms share the cached posterior, measurements, link uniforms, filter RNG,
static carrier graph, fusion weights, three-step horizon and communication
constraints.  Positive percentages mean lower E-OSPA than the named control.

| Scale | t | V96 set | V97 set | Static | V96 | V97 | V97/static | V97/V96 | Gate |
|:--|--:|:--|:--|--:|--:|--:|--:|--:|:--:|
| M24 | 104 | [1 3] | [1 3 4] | 71.664511 | 67.229679 | 65.770429 | +8.225% | +2.171% | pass |
| M24 | 124 | [2 3 4] | [1 2 3 4] | 83.582917 | 77.559560 | 76.841448 | +8.066% | +0.926% | pass |
| X36 | 72 | [1 2 4] | [1 2 4 5] | 85.970277 | 84.623645 | 83.896827 | +2.412% | +0.859% | fail |
| X36 | 100 | [4 5 6] | [1 3 4 5 6] | 89.375579 | 84.946782 | 82.964034 | +7.174% | +2.334% | pass |

V97 also has nonnegative worst-sensor and weakest-formation gains at all four
anchors.  Its consensus gains are 18.576%, 16.267%, 4.273% and 12.011%, while
attempted-byte savings are 3.950%, 6.452%, 5.305% and 6.127%.

## Interpretation

Selecting all initially safe positive-net formations is useful: it improves
every anchor over the V96 minimum 80%-cover set and raises X36 t=100 above the
5% gate.  The X36 t=72 failure shows that insufficient initial spatial
coverage is not the whole problem.  At t=72, formations 3 and 6 are excluded
because their single-step counterfactuals cause a cross-supported downward
decision crossing; V97 already selects every other positive-net formation.
No larger safe static set exists under the current action family at that
initial state.

The next experiment should therefore inspect the candidate posterior at the
second and third steps.  If formations 3 or 6 become addressable, or positive
net risk moves among the other formations, the successor should recompute the
safe positive-net set online at every step.  If the same two formations remain
unsafe and no risk migration appears, re-selection cannot close the gap and
the action family must be expanded.  Do not tune the 80% target, add an
identifier-specific exception or open richer scenes before this distinction is
resolved.

This is opened development evidence.  It supports a current-state spatial
coverage mechanism, not a full-episode, cross-scene or dynamic-physical-route
claim.
