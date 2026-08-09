# V64: cumulative observable-risk coverage

## Method decision

A fixed top-k is not scale invariant.  Protecting three of four M24 formations
and three of six X36 formations changes the fraction of the network directly
affected, even if the per-formation mechanism is identical.  Conversely,
protecting a fixed fraction of formations ignores whether current risk is
concentrated or diffuse.

V64 instead treats each formation's nonnegative, association-supported rescue
score as observable risk mass.  It sorts formations by that mass and selects
the smallest prefix whose cumulative sum reaches `80%` of total positive mass:

`k* = min { k : sum_(i<=k) s_(i) / sum_i s_i >= 0.80 }`.

The selected formations keep the registered physical carrier graph and
control synopses but do not consume complete cross-formation posteriors for
all three H=3 steps.  No formation identifier, node count, truth value, future
measurement, or prior tracking outcome enters the selector.

## Why 80%

The threshold represents a coverage requirement rather than a formation-count
parameter.  It is frozen before the V64 outcome.  On the already observable
X36 states it produces different set sizes:

| State | Selected prefix | Achieved risk coverage |
|:--|:--|--:|
| t=72 | `[4,2,3,5]` | 82.4% |
| t=100 | `[6,4,5,2]` | 83.0% |
| t=128 | `[3,4,5]` | 93.3% |

This is the intended behavior: diffuse risk activates four formations, while
concentrated risk activates only three.

## Gate

The first screen compares exactly two arms at X36 seed 211, t=72: registered
full-payload reference and the frozen 80%-coverage persistent schedule.  The
candidate must reach at least `5%` mean tracking gain while worst-sensor,
minimum-formation, window consensus, terminal consensus, attempted bytes, and
rolling B3 connectivity all remain nonnegative.  Failure stops before t=100
and t=128; passing opens those two states but still does not authorize model
training or validation claims.

## Opened development outcomes

The frozen selector passed both opened X36 states evaluated so far:

| State | Selected set | Mean tracking | Worst sensor | Window consensus | Terminal consensus | Attempted bytes |
|:--|:--|--:|--:|--:|--:|--:|
| t=72 | `[4,2,3,5]` | `+5.847%` | `+27.843%` | `+15.719%` | `+17.567%` | `+3.623%` |
| t=100 | `[6,4,5,2]` | `+9.329%` | `+11.723%` | `+21.467%` | `+22.966%` | `+4.533%` |

Both rows pass the registered strict and 5% mean-gain gates.  These are two
opened states from one X36 seed, so they establish repeatable development
headroom but not cross-seed or cross-scene generalization.
