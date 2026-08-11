# Task-aligned label value direction after receiver-granularity check

## Stage 0: do not assume label granularity prematurely

V103 proves that formation-wide handoff is unsafe, but receiver selection may
still be sufficient.  V104 therefore runs one retrospective receiver oracle
before opening label combinations.  It keeps only V103 receiver rows with
positive same-page paired tracking gain.  If this outcome-informed upper bound
passes the strict gate, the deployable model remains a receiver-edge value
predictor.  The label-wise stages below open only if V104 fails.

## Decision object

V104 keeps the physical route and the V101/V103 protection-age state machine.
For each matured gateway handoff, it replaces the formation-wide binary action
with one label-wise participation variable

```text
z(receiver, gateway, label, time) in {reference input, gateway input, both}
```

The physical graph determines what can be sent.  The formation controller
determines when a gateway posterior is mature.  The label-value layer
determines whether that posterior is useful to a particular receiver-label.
This hierarchy preserves the demonstrated temporal mechanism while removing
the coarse action that made F1 and F6 unsafe.

## Stage A: bounded exact headroom teacher

Before fitting a model, enumerate feasible sender choices for each
receiver-label on the three V103 handoff pages.  Options are evaluated jointly
within a receiver because KLA normalization couples sender choices for the
same label.  Candidate subsets must satisfy:

1. no downward extraction crossing for a receiver-supported label;
2. no decrease of its supported existence below the registered log-odds
   retention floor;
3. complete GM payloads for every selected label;
4. attempted bytes no larger than the matched static row budget;
5. unchanged physical carrier and rolling B3 reserve.

Offline short rollouts score feasible options by downstream E-OSPA and
cardinality regret.  Future truth is used only for teacher targets, never as a
candidate feature.  The first oracle bank is deliberately bounded to the
receivers actually changed by V103 and to active labels present in the
reference or matured gateway posterior.

The exact headroom gate is unchanged in spirit: at least 5% mean improvement,
nonnegative minimum receiver/formation and consensus tails, nonincreasing
attempted bytes, and at least 1% gain for the previously weak F6 non-gateway
group.  It must pass one M24 and one X36 development window before learning.

## Stage B: set-GNN approximation

If exact headroom exists, train a permutation-invariant set GNN to predict
option regret rather than edge class or posterior-preservation KLD.  Causal
features include receiver/gateway/incumbent existence, association support,
GM complexity, moment disagreement, covariance-normalized innovation, FoV
opportunity, link quality, protection age and recent selected/delivered
history.  Target identity, future measurements and future delivery outcomes
are excluded.

The GNN ranks feasible options; it does not enforce safety.  The deterministic
projector applies the five constraints above and falls back to the reference
label input whenever confidence is low or no candidate is feasible.  This
separates the data-driven value estimate from physical, communication and
estimator invariants.

## Generalization gate

After freezing the model and projector, test matched static comparisons on:

- M24 and X36 radial development windows not used for teacher fitting;
- convoy and relay scenes;
- merge-split and curved-corridor family holdouts;
- at least one larger sensor-count setting.

Report tracking, cardinality, minimum formation/receiver, consensus,
attempted and delivered bytes, model fallback rate and oracle regret.  A model
that wins only on network mean or only on radial scenes does not advance.
