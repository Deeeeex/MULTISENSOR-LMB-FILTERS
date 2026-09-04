# V261 risk-triggered formation shortcut

## Why the action scale changed

V260 added a complete local residual bundle inside the formation selected by
the V259 localization-risk gate.  It improved E-OSPA and inter-formation
consistency, but its stronger arm caused a larger RMSE regression.  The
pre-topology V259 state explains why: for label `[1,4]` at `t=57`, all six F4
nodes have nearly the same mean and all are about 60 units from truth.  Their
within-formation spread is small relative to the shared error, so another
F4-to-F4 input cannot remove the common-mode bias.

| Formation at t=57 | Median existence | Median position error | Median covariance trace |
|--:|--:|--:|--:|
| F1 | 0.956 | 13.4 | 217 |
| F2 | 0.855 | 34.6 | 2,154 |
| F3 | 0.311 | 55.4 | 5,054 |
| F4 | 0.629 | 61.3 | 6,140 |

The V242 formation tree at this page is `F1--F2--F3--F4`, although the current
physical formation graph also contains `F1--F3`.  The best-supported copy of
the risky F4 label is therefore three KLA hops away under V242 but only two
physical hops away through `F1--F3--F4`.

## Registered causal action

V261 keeps V242 unless all of the following hold:

1. one formation passes the frozen V259 localization-tail gate;
2. another formation has at least 0.80 label coverage, median existence at
   least 0.50, and no more than half the target formation's label-localization
   risk;
3. the current physical formation graph offers at least one fewer hop between
   that donor and target than the current V242 tree.

When these conditions hold, V261 requests the reliable shortest donor-target
path, retains as many current tree edges as possible without creating a cycle,
and completes the tree by current link reliability.  The existing V249/V242
projection then chooses physical sensor gateways and reconstructs the ordinary
mixture-aware LMB-KLA route.

At the initial `t=57` state, the registered rule selects F1 as donor, F4 as
target, label `[1,4]`, and changes
`F1--F2--F3--F4` to `F2--F1--F3--F4`.  The projected route is physical,
strongly connected, and still uses exactly 30 directed posterior messages.

## Evidence boundary and stopping rule

The online selector reads current posterior summaries, physical links and past
selected routes, but no truth, future measurements or future outcomes.  It is
still a centralized development controller and its synopsis traffic is not
charged.  The first experiment is only the paired `t=57--73` continuation from
the exact V259/V242 state.  Continue to a full M24 episode only if F4 event
E-OSPA and RMSE both improve, network E-OSPA/RMSE/consistency remain within the
registered guards, no formation regresses by more than 3%, and the spliced
full-episode posterior traffic remains below the static baseline.
