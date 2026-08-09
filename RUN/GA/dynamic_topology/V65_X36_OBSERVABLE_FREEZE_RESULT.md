# V65 X36 observable freeze result

V65 was frozen before the X36 t=128 tracking outcome was opened.  It measures
receiver-supported rescue and cross-supported useful loss using one network
reference-existence denominator, then applies the registered 1% risk budget,
80% coverage requirement, useful-loss guard, and zero supported downward
crossing constraint.

| State | Network risk | Useful loss | Frozen action | Coverage | Prior outcome status |
|:--|--:|--:|:--|--:|:--|
| t=72 | `1.512%` | `0.050%` | `[2,3,4,5]` | `81.126%` | same schedule already gave `+5.847%` |
| t=100 | `2.267%` | `0.016%` | `[2,4,5,6]` | `81.846%` | same schedule already gave `+9.329%` |
| t=128 | `0.428%` | `0.006%` | reference fallback | -- | outcome remains unopened |

All three states have zero cross-supported downward decision crossings.  The
result gives the desired scale-aware behavior: the rule acts in the two
high-risk states with independently observed strong headroom and declines to
intervene in the low-risk state.  It does not establish held-out or cross-scene
generalization; the next required check is the same frozen rule on M24 strong
and weak development states.
