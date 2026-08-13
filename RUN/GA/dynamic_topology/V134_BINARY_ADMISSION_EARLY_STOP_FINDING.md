# V134 binary posterior-admission early-stop finding

## Decision

V134 is stopped before learner development.  The complete M24 action bank
contains no action that passes the preregistered pre-learning gate, so the
joint M24/X36 gate is already logically impossible.  Completing the remaining
X36 combinations cannot change that decision and would only enlarge a failed
screen.

This file is a repository experiment record.  The below-gate outcomes are not
promoted to the main progress document.

## Frozen comparison

- Source: `cc6b5d673bed635931b0769255737d533e87b162`
- Baseline source: `b207115f9010af6c4dac067e34efbdb27a2de1d1`
- M24 reference: fixed counter-clockwise, 2.351% better than clockwise over
  the four complete development trajectories.
- X36 reference: fixed counter-clockwise, 0.401% better than clockwise over
  the same four development seeds.
- M24: all 12 singleton, pair-complete and high-order-prefix actions were
  evaluated.
- X36: all six singleton actions and three pair actions were evaluated before
  the logically decisive early stop.  The X36 screen is intentionally not a
  complete action-bank characterization.

## Decisive evidence

1. M24 contains real short-horizon headroom.  The all-formation action reaches
   +8.553% intervention E-OSPA gain, while the rank-1/rank-3 pair reaches
   +5.066% intervention gain and remains positive on average over the full
   and recovery windows.
2. No M24 action is safe at sensor level.  The rank-1/rank-3 pair passes the
   aggregate, recovery, formation, byte and structural gates, but its worst
   recovery sensor regresses by 2.976%.  That sensor is the cross-formation
   input gateway of formation 4.
3. Stronger protection increases the long-term debt.  The all-formation M24
   action changes from +8.553% during intervention to -4.163% in the fully
   restored recovery tail.
4. The structural recovery certificates pass while tracking can regress.
   Internal KLA disagreement contracts after full restoration, but the network
   can converge around a posterior trajectory displaced from the static
   reference.  More consensus rounds do not remove this common-mode bias.
5. X36 singleton actions provide at most small intervention gains in the
   completed screen, and five of six increase attempted bytes because the
   fixed control synopsis can exceed the payload it replaces.  The first three
   pair actions retain the same short-positive/recovery-negative pattern.

## Method consequence

The next method must not optimize only the omitted formation set or the length
of the full-admission tail.  It must separate two roles of a gateway posterior:

- the state used for the gateway's own protected estimate;
- the state relayed to downstream formations.

A causal next experiment should maintain an outward reference lineage without
using truth or a paired alternative-arm state, use it only on cross-formation
relay edges, and charge every auxiliary payload.  It also needs a payload-aware
fallback: an admission action is ineligible whenever its control metadata is
not smaller than the posterior transmission it replaces.

The immediate V135 question is whether a locally protected estimate can retain
V134's intervention gain while a separately maintained outward relay state
prevents the gateway bias from propagating.  Only a paired M24/X36 headroom
screen can authorize a learner.

## Evidence locations

- Frozen carriers:
  `RUN/GA/dynamic_topology/evidence/tracking_aligned_v133/counterfactual_regret_gate/baseline_selection/FROZEN_REFERENCE_CARRIER_V133.md`
- V134 checkpoints:
  `RUN/GA/dynamic_topology/evidence/tracking_aligned_v134/binary_admission_sequence_v4/pilot/`
- Worker logs:
  `RUN/GA/dynamic_topology/evidence/tracking_aligned_v134/binary_admission_sequence_v4/logs/`

