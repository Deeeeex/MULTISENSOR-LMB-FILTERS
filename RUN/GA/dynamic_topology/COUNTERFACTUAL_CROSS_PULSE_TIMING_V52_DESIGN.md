# Counterfactual cross-pulse timing V52

## Method decision

V51 asks only whether the synchronized V46 cross-formation pulse should be
deferred. That is a useful falsification arm, but it has two limitations. Its
existence-gap score is only a fast proxy for the actual LMB fusion outcome,
and its action space can only remove information. The latter is particularly
restrictive on X36, where the large cardinality error may require useful label
evidence to propagate earlier rather than less often.

V52 therefore treats the communication time, rather than only the graph, as
the decision. It retains the repaired V46 dominant route and the same B4
posterior-message budget. The primary action moves the complete residual
layer as one synchronized pulse: every four-step service window executes
exactly one current, repaired V46 residual page, but a causal controller may
choose when to execute it. The dominant layer remains active on every page.
The primary method therefore still uses exactly `5N` attempted posterior
messages per four steps, and its executed pulse is a complete strongly
connected V46 route rather than a mixture of independently timed formation
fragments.

This global pulse is deliberate. The repaired V46 sender attached to a
receiver can change between pages. Independently moving each formation's
bundle would therefore not, by itself, preserve either the exact message
count or the connected V46 route union. Formation-specific service remains a
possible extension only after a causal budget-and-connectivity projection is
defined; it is not part of the primary V52 claim.

## Counterfactual score

While the current four-step pulse credit remains unused, the controller
compares two one-round outcomes under the current posterior and current link
delivery probabilities:

1. serve the complete current repaired residual layer with weight 0.20;
2. hold the pulse on this page and execute the dominant layer only, returning
   the omitted residual weight to self.

Both outcomes execute the same fusion operator and missing-message weight
normalization as the tracking runtime. Their difference supplies three
truth-free quantities: supported-label retention, expected-cardinality
change, and receiver-to-network posterior disagreement. A service is
preferred when it supplies complementary existence evidence or reduces
disagreement without materially diluting labels already supported by the
receiver. A hold is preferred when current fusion would reduce that support.

The decision needs only these two route evaluations on an eligible page. Its
action count is independent of formation count and it does not enumerate
graphs, cycles, phase vectors or formation subsets. Formation-wise mean and
tail effects are retained in the score so that a favorable network average
cannot hide severe dilution in one formation.

## Causal service rule

Simulation time 1 executes the ordinary synchronized pulse as bootstrap.
Each later service window is the four-page interval after one pulse and up to
the next registered V46 pulse: absolute phases 2, 3, 4 and then 1. On phases
2--4, execute early only when the current counterfactual benefit passes the
fixed retention and disagreement gates; otherwise keep the single pulse
credit. On the closing phase 1, execute any unused credit. The decision uses
current and past observable states only. It does not use target truth, future
measurements, future geometry or realized packet delivery.

This is a causal optimal-stopping problem under a hard service deadline. The
deterministic first implementation uses the exact one-round score as a myopic
stopping rule. A later data-driven model may estimate the value of waiting
from recent posterior and link trends, but it may only propose serve/hold
scores. The one-pulse ledger, current physical-edge projection and hard
four-page deadline are deterministic runtime constraints. Because every
window contains one complete V46 residual pulse, its union contains a full
strongly connected V46 route and requires no predicted future topology.

## Why this differs from earlier branches

- V48 placed residual service using graph contraction only and found less
  than 1% structural headroom on X36. V52 uses the estimator state and asks
  whether the message is useful now.
- V49 changed the cross-formation route and worsened X36 tracking despite a
  better graph score. V52 keeps the repaired V46 route.
- V50 replaced senders but selected no nonreference route in 40/40 windows.
  V52 changes service time rather than sender identity.
- V51 can defer one synchronized pulse and reduce communication. V52 instead
  preserves the V46 message budget and can move a useful complete pulse
  earlier while delaying a currently harmful pulse only to its hard deadline.
- V30/V35 established that actual serve/hold fusion counterfactuals contain
  useful M24 signal. V52 retains that estimator-side score while replacing
  their state-specific three-step suspension with a scale-independent single
  B4 pulse ledger.

## Experiment branch

The running V51 X36 convoy result decides the immediate branch:

- material tracking and cardinality improvement: keep V51 as the low-cost
  arm, then compare V52 only if the remaining X36 error is still large;
- no deferrals or a neutral result: reject the existence-gap proxy and move
  directly to V52;
- frequent deferrals with worse tracking: reject removal-only control and use
  V52's fixed-budget timing action.

The first V52 experiment, if opened, uses the saved X36 convoy seed-1009 V46
baseline and runs only the candidate. A positive direction must then recur on
X36 merge-split and curved-corridor before the unchanged method is evaluated
on M24. Training a GNN is downstream of this analytic causal baseline, not a
substitute for demonstrating that the action space has estimator value.
