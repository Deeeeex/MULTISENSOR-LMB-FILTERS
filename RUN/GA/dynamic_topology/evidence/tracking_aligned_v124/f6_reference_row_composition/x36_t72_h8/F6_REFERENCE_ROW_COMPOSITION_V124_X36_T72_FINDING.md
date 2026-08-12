# V124 finding: reference rows do not restore reference sender state

## Frozen comparison

V124 keeps the V123 payload schedule and all projected formation rows except
that F6 uses the fixed-clockwise reference row on all eight pages.  The real
X36 preflight preserved 60 messages per page, physicality, the reference
weight multiset, and sensor/formation rolling-B3 connectivity.

| Arm | Gain vs clockwise | F5 gain | F6 gain | F6 terminal gain | Gate |
|---|---:|---:|---:|---:|:---:|
| V123 projected rows | +3.079% | +9.913% | -6.894% | -7.244% | fail |
| V124 F6 reference row | +3.973% | +9.913% | -1.495% | -5.865% | fail |
| V122 release F5 | +2.711% | -0.014% | approximately 0% | approximately 0% | fail |

V124 improves the aggregate by 0.894 percentage points over V123 and recovers
5.399 percentage points of F6 window-average performance.  It nevertheless
misses the 5% aggregate gate, the mature-page floor is only +2.026%, and F6
peers regress by 7.206% at the terminal page.

## Method decision

The residual F6 debt cannot be removed by restoring only the receiver row.
F5 has already evolved under the protected payload schedule; the fixed
reference F5-to-F6 edge therefore carries a non-reference sender posterior.
V122 confirms the other side of the trade-off: restoring F5 itself protects
F6, but removes the useful F5 local effect.

This closes independent formation-row composition as the next optimization
axis.  The next action representation must distinguish the posterior used for
local estimation from the state relayed downstream.  A bounded oracle should
first test whether an outward reference carrier on the F5-to-F6 boundary can
retain the F5 local gain without exporting its delayed debt.  Only if that
oracle clears the frozen X36 gate is a causal lineage-aware or dual-state relay
worth implementing as a deployable method.

This result is an opened-development diagnostic and is not validation or
generalization evidence.
