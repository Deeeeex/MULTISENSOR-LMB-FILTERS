# Counterfactual cross-pulse timing V52

## Method decision

V51 asks only whether the synchronized V46 cross-formation pulse should be
deferred. On X36 convoy seed 1009 it deferred 34 of 40 pulse opportunities,
saved 1.43% of attempted messages and improved cardinality and consensus by
1.51% and 1.59%, but its full-horizon E-OSPA improvement was only 0.01%.
This is a useful directional signal rather than a successful method. Its
existence-gap score is only a fast proxy for the actual LMB fusion outcome,
and its action space can only remove information.

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
Every service window then follows the ordinary V46 period boundary: absolute
phases 1, 2, 3 and 4. On phases 1--3, execute the pulse when the current
counterfactual benefit passes the fixed retention, cardinality and
disagreement gates; otherwise retain the single pulse credit. On phase 4,
execute any unused credit. Once a pulse has been served, the remaining pages
of that window execute only the dominant layer. The decision uses current and
past observable states only. It does not use target truth, future
measurements, future geometry or realized packet delivery.

This alignment is informed by the V51 result. V51 obtained its small benefit
by withholding the ordinary phase-1 pulse. A phases-2--4--1 window would
still force that same potentially harmful phase-1 pulse and therefore could
not preserve the observed direction. A phases-1--4 service window instead
allows the complete pulse to be delayed, re-evaluated and restored later
without deleting its communication budget.

This is a causal optimal-stopping problem under a hard service deadline. The
deterministic first implementation uses the exact one-round score as a myopic
stopping rule. A later data-driven model may estimate the value of waiting
from recent posterior and link trends, but it may only propose serve/hold
scores. The one-pulse ledger, current physical-edge projection and hard
four-page deadline are deterministic runtime constraints. Because every
window contains one complete V46 residual pulse, its union contains a full
strongly connected V46 route and requires no predicted future topology.

## Closest prior work and boundary

Event-triggered consensus LMB filters decide whether a node or Bernoulli
component should be broadcast by comparing the current density with the most
recent broadcast or its prediction, commonly through a KL-divergence
threshold. They reduce update frequency or payload on a given neighbor graph;
they do not select the position of one mandatory full-posterior pulse inside a
fixed communication window.

Value-of-Information schedulers for networked state estimation are closer to
V52's estimator-side decision: they prioritize packets by their expected
effect on estimation accuracy under a shared-channel budget. Predictive and
self-triggered estimation further shows why an instantaneous event trigger
can waste slots that the communication system cannot reallocate in time.
Learning-based schedulers such as SchedNet show that message importance can be
learned from task outcomes under bandwidth constraints.

V52 combines these ideas at a different deployment surface. Its analytic
baseline evaluates the actual current LMB serve/hold fusion counterfactual,
its service ledger fixes one complete residual pulse per B4 window, and its
current V46 projection supplies a physical strongly connected route. A later
learner estimates only the value of waiting; it cannot change the pulse
budget, fusion operator or executable topology. The related references are:

- K. Shen et al., "Consensus-Based Labeled Multi-Bernoulli Filter With
  Event-Triggered Communication," IEEE TSP, 2022,
  https://doi.org/10.1109/TSP.2022.3154227.
- A. Molin, H. Esen and K. H. Johansson, "Scheduling networked state
  estimators based on Value of Information," Automatica, 2019,
  https://doi.org/10.1016/j.automatica.2019.108578.
- S. Trimpe, "Predictive and Self Triggering for Event-based State
  Estimation," 2016, https://arxiv.org/abs/1609.07534.
- S. Wu et al., "Learning Optimal Scheduling Policy for Remote State
  Estimation under Uncertain Channel Condition," IEEE TCNS, 2020,
  https://doi.org/10.1109/TCNS.2019.2959162.
- D. Kim et al., "Learning to Schedule Communication in Multi-agent
  Reinforcement Learning," ICLR, 2019,
  https://arxiv.org/abs/1902.01554.

## Why this differs from earlier branches

- V48 placed residual service using graph contraction only and found less
  than 1% structural headroom on X36. V52 uses the estimator state and asks
  whether the message is useful now.
- V49 changed the cross-formation route and worsened X36 tracking despite a
  better graph score. V52 keeps the repaired V46 route.
- V50 replaced senders but selected no nonreference route in 40/40 windows.
  V52 changes service time rather than sender identity.
- V51 can defer one synchronized pulse and reduce communication. V52 instead
  preserves the V46 message budget and can delay a currently harmful phase-1
  pulse while restoring one complete pulse by the phase-4 hard deadline.
- V30/V35 established that actual serve/hold fusion counterfactuals contain
  useful M24 signal. V52 retains that estimator-side score while replacing
  their state-specific three-step suspension with a scale-independent single
  B4 pulse ledger.

## Experiment branch

V51 is retained only as evidence that harmful phase-1 fusion exists. Its
0.01% full-horizon E-OSPA gain is far below the intended material effect, so
its proxy threshold is not tuned further. The immediate branch is V52.

The first V52 experiment uses the saved X36 convoy seed-1009 V46 baseline and
runs only the candidate. A positive direction must then recur on X36
merge-split and curved-corridor before the unchanged method is evaluated on
M24. Training a GNN is downstream of this analytic causal baseline, not a
substitute for demonstrating that the action space has estimator value.
