# V81 predictive soft-return recovery finding

V81 rejects intervention-local residual trust as the missing cross-scale
recovery action.  The same predictive bank selects the expected small global
correction on the first X36 recovery round, but no action removes the complete
two-round rebound, and M24 has no contracting first recovery action at all.

| Case | Selected sequence | Centered energy by round | Monotone |
|:--|:--:|:--|:--:|
| M24 historical | `R-R` | `0.005036, 0.005577, 0.004053` | no |
| M24 aligned | `R-R` | `0.004001, 0.004213, 0.004165` | no |
| X36 historical | `G02-R` | `0.001798, 0.001437, 0.002900` | no |
| X36 aligned | `G05-R` | `0.001987, 0.001459, 0.002911` | no |

The M24 result is stronger than a greedy-search failure.  Every action in the
bank increases centered energy in the first recovery round relative to the
safe pulse.  No later sequence over this bank can satisfy the frozen monotone
condition because its first transition has already failed.  Ordinary
reference recovery is nevertheless the least harmful action.

The X36 result preserves the V80 mechanism.  `G02` and `G05` improve the first
recovery state, but at the second recovery state ordinary reference is again
the minimum-risk member and still rebounds.  A larger global correction or a
local soft return only increases the terminal energy.

Soft return fails because it acts at the two receivers directly modified by
the pulse after the perturbation has already propagated through the dominant
within-formation star.  Across all four route cases, reducing the restored
`0.05` incumbent input is worse than full reference restoration.  The next
action must therefore target the nodes that currently carry growing centered
existence debt, not the nodes where the original residual replacement was
made.

V82 should add a node-resolved decomposition whose contributions sum exactly
to V77 centered energy.  At each recovery state it should compare the current
candidate arm with the virtual ordinary-reference next state, rank positive
per-node contribution growth, and nominate a small fixed number of propagation
receivers.  Only those receivers should receive small dominant-input damping,
with the removed weight returned to self.  Candidate rows and damping levels
must then be evaluated by the same causal virtual KLA objective.  This creates
a direction-aware sparse correction while preserving reference senders and
message count.

V81 is opened-anchor source-only evidence.  It does not establish tracking
harm or gain and uses no truth, new measurement, future link, packet draw,
route execution, outcome, or model training.
