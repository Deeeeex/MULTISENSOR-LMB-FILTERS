# Formation reference-recovery horizon v25 audit

## Verdict

v25 passed its protocol, source-provenance, H=3 reproduction, exact-execution,
payload, selected rolling-B3, truth-use, repair, emergency, and feasibility
checks.  It failed both the strict and strong recovery gates at H=4 and H=5.
One or two passive fixed-reference steps do not repair the consensus debt of
the frozen high-return prefixes.

## Contract checks

- generation commit: `27b2c879c634a4e32705c53cf413caa7df9fb6d1`;
- opened state: `m24-formation-fov / seed 211 / t=72`;
- frozen candidates: four v21/v24 three-step prefixes;
- recovery: one reference step at H=4 and two reference steps at H=5;
- control: an all-reference trajectory of the same length at each horizon;
- H=3 prefix reproduction maximum error: `3.55e-15` percentage points;
- strict feasibility: all seven signed targets nonnegative;
- strong safety: strict feasibility plus at least `3%` mean tracking gain.

The seventh target is final-step consensus.  The fourth target remains
consensus averaged over the complete equal-length window.  This distinction
prevents a longer horizon from appearing safe merely by averaging away an
unrecovered final state.

## Equal-horizon outcomes

| H | Candidate | Mean | Min. formation | Worst sensor | Window consensus | Attempted | Delivered | Final-step consensus | Strict |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| 4 | 1 | +6.728% | 0 | +0.055% | -3.603% | -0.422% | -0.440% | -10.094% | no |
| 4 | 2 | +7.128% | 0 | +0.082% | -1.883% | +0.224% | -0.424% | -2.444% | no |
| 4 | 3 | +9.293% | 0 | -1.635% | -3.158% | -0.339% | -1.009% | -1.519% | no |
| 4 | 4 | +4.258% | 0 | -0.005% | -2.325% | +0.040% | +0.042% | -4.357% | no |
| 5 | 1 | +9.091% | 0 | +1.400% | -5.038% | -0.758% | -0.801% | -11.147% | no |
| 5 | 2 | +9.735% | 0 | +1.400% | -3.282% | -0.079% | -0.624% | -9.238% | no |
| 5 | 3 | +11.289% | 0 | +1.400% | -3.810% | -0.835% | -1.400% | -6.589% | no |
| 5 | 4 | +4.740% | 0 | -1.619% | -2.800% | -0.128% | -0.157% | -4.824% | no |

The strict and strong counts were `0/4` at both horizons.  Candidate 2 is the
most useful H=4 boundary point: it improves mean tracking, the two tail
criteria, and attempted bytes, but not delivered bytes or either consensus
criterion.  No H=5 candidate preserves a communication saving.

## Research finding

The H=3 advantage is not a fixed reward followed by a fixed recovery cost.
Changing the topology changes the posterior path, which in turn changes later
tracking errors, disagreement, and payload sizes even after the fixed
reference graph is restored.  The final-step deficits becoming larger at H=5
show that passive reference replay is not a contraction mechanism for these
branch states.

This closes the "extend the window and wait" mechanism.  The result does not
close dynamic topology as a whole.  A deployable dynamic policy can observe
the posterior and innovation state at every scan, whereas all v21-v25 actions
were selected from the opened state at t=72 and then frozen.  Any next action
family must therefore be closed loop and state conditioned.

## Next decision

First audit the exact fusion-weight products.  Rolling-B3 guarantees that the
recent union graph is strongly connected, but it does not by itself guarantee
rapid contraction of KLA log-density disagreement.  Measure sensor-level and
formation-level mixing coefficients for the all-reference and four recovery
sequences and compare them with the realized consensus targets.

If a topology-only coefficient separates safe from unsafe trajectories, add
it as a hard projection constraint.  If it saturates or fails to rank the
outcomes, stop treating graph structure as a sufficient safety state and
design a posterior/innovation-aware multi-head value model with a registered
reference fallback.  No GNN training is authorized by this audit alone.

## Evidence boundary

All findings are privileged mechanism evidence from seed 211 at t=72.  Seeds
223/227, X36, and final seeds remain unopened.  The result cannot support
generalization, learned-policy, or paper-level performance claims.
