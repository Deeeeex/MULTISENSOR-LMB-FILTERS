# Formation reference-recovery horizon v25

## Question

The best v21/v24 three-step sequences already improve tracking, both tail
criteria, and both communication criteria, but their consensus error averaged
over the three-step window remains `1.5%--1.7%` worse than the fixed reference.
The existing result does not show whether the nodes are still diverging at the
third step or are already recovering after a transient disturbance.  v25 asks
whether one or two additional reference-topology steps remove both the
window-average and final-step consensus debt without erasing the other gains.

## Reference and recovery action

The reference action is mode vector `[1,1,1,1]`.  At that step the runner
recomputes the registered fixed counter-clockwise directed topology and its
fusion weights.  It does not keep the preceding dynamic graph.  The
all-reference control applies this same action at every step of the comparison
window.

Every candidate uses a frozen three-step active prefix selected from v21/v24,
then returns to the reference action:

- H=4: three active steps followed by one reference recovery step;
- H=5: the same three active steps followed by two reference recovery steps.

Each candidate is compared only with an all-reference trajectory of the same
length, so a longer window cannot receive an artificial advantage from a
shorter communication or tracking denominator.

## Frozen candidates

| ID | Role | Three-step active prefix | Registered H=3 targets |
|--:|:--|:--|:--|
| 1 | positive tails and lowest saved consensus debt | `[1,1,3,1] -> [1,1,1,4] -> [1,4,4,2]` | `[+5.774941, 0, +0.042126, -1.527435, +1.027062, +1.073903]%` |
| 2 | balanced high return | `[1,4,3,1] -> [1,1,1,4] -> [1,1,4,2]` | `[+5.752299, 0, +0.100275, -1.704093, +1.809861, +1.011454]%` |
| 3 | maximum mean headroom | `[1,4,3,1] -> [1,1,1,4] -> [1,1,1,2]` | `[+8.648597, 0, +0.100275, -3.681700, +1.020828, +0.186436]%` |
| 4 | low-return boundary with small sensor debt | `[1,1,2,2] -> [1,4,1,1] -> [1,1,4,1]` | `[+3.891414, 0, -0.002126, -1.675203, +0.161489, +0.168854]%` |

The six H=3 target columns are, in order: mean tracking gain, minimum
formation gain, worst-sensor gain, consensus gain averaged over the window,
attempted-byte saving, and delivered-byte saving.  Candidate selection uses
opened future outcomes and is therefore a mechanism probe, not a deployable
policy.

## Reproduction and execution gates

- the first three steps embedded in both H=4 and H=5 must reproduce each
  candidate's registered H=3 target vector within `5e-6` percentage points;
- the reference row must be exactly zero under each equal-horizon comparison;
- every nonreference graph must execute exactly as frozen;
- all selected graphs must pass sensor-level and formation-level rolling-B3;
- no runtime truth use, repair fallback, payload emergency, or infeasible
  topology is allowed;
- every action must remain inside the registered reference payload bound.

Delivered rolling-B3 is recorded but is not a hard gate because stochastic
packet drops can disconnect the realized delivery graph even when the selected
communication graph is safe.  Its effect is captured by delivered bytes and
the two consensus targets.

## Decision gate

The equal-horizon score has seven signed targets: the six H=3 targets plus
final-step consensus gain.  Strict feasibility requires all seven to be
nonnegative.  Strong safety additionally requires at least `3%` mean tracking
gain.

If either H=4 or H=5 contains a strong-safe candidate, the temporal recovery
mechanism is viable and broader opened-state teacher collection may begin.
This still does not authorize GNN training or validation; the strong pattern
must first recur across multiple development states.  If neither horizon
passes, further H=3 mode-vector enumeration remains closed and the action
representation must be redesigned or abandoned.

Seeds 223/227, X36, and final seeds remain unopened.

## Result

The official run at generation commit `27b2c87` reproduced every embedded
H=3 prefix with a maximum target error of `3.55e-15` percentage points.  The
horizon generalization therefore preserves the registered three-step
experiments.

Neither H=4 nor H=5 contained a strictly feasible or strong-safe candidate.
The closest H=4 sequence was candidate 2, with targets
`[+7.128027, 0, +0.082042, -1.883399, +0.223614, -0.424018,
-2.444057]%`.  It retained substantial tracking gain and a small attempted
byte saving, but failed window-average consensus, delivered bytes, and
final-step consensus.  At H=5, every candidate retained positive mean
tracking gain, but all four lost both window-average and final-step consensus;
their final-step consensus deficits ranged from `-4.824%` to `-11.147%`.

Appending the fixed reference topology therefore does not repay the state
divergence created by these prefixes.  It can also reverse the H=3
communication saving because the altered posterior trajectory changes later
payload sizes.  This rejects the specific "short window plus passive reference
recovery" explanation.  It does not reject a state-conditioned recovery
policy that observes the updated posterior and selects a new action online.

Broader teacher collection, GNN training, reserved-seed evaluation, and X36
evaluation remain unauthorized.  Before designing a closed-loop action, the
next diagnostic must test whether rolling-B3 and topology-only mixing
coefficients explain the observed consensus debt.  If they do not, the value
model and safety layer must include posterior/innovation state rather than
graph structure alone.
