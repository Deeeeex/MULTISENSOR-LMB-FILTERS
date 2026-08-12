# V133: pre-send counterfactual regret gate

## Research question

The current X36 evidence says that receiver-side posterior protection has useful aggregate headroom, but repeated protection can accumulate local state error and later harm downstream peers. A current-only rule cannot distinguish a temporarily useful action from an action whose debt will become visible several fusion pages later. V133 therefore asks a narrower causal question before every send: if one receiver formation withholds its incoming cross-formation posterior on this page and the network returns to the frozen full-posterior carrier afterwards, will that single action create harmful regret over one information-propagation horizon?

## Causal action and reference

The action changes no physical or logical topology. The same carrier edge is attempted and its control synopsis is charged, but the full posterior does not enter fusion. The runtime calls this `abstention-only`: “control-only” describes the bytes on the wire, whereas `abstention-only` additionally guarantees that an empty delivered payload is not interpreted as negative label evidence. All remaining inputs follow the runtime missing-neighbor rule. Under `renormalize`, their original weights are normalized after the protected input is removed; under `self`, the protected weight is transferred to the receiver. This is the actual payload-abstention semantics and differs from the older V65 whole-formation topology-suspension proxy.

Development runs compare fixed clockwise and fixed counter-clockwise full-posterior carriers on complete trajectories. The lower paired mean E-OSPA carrier is then frozen before calibration. Per-state future outcomes are never used to switch the fallback carrier online.

## State representation

The first model is deliberately interpretable. Its inputs combine:

- exact one-page expected posterior-risk and label-retention changes under the actual action;
- mixture-aware sender/receiver label differences, including existence, precision, spatial compatibility, association support and multimodality;
- current observation quality and short update trends for the affected formation;
- current link opportunity and past delivery acknowledgements on the protected edge;
- consecutive protection age, recent duty cycle and time since release;
- the number and distance of nodes and formations that can inherit the state change over the registered propagation horizon.

Truth, E-OSPA, future measurements, future delivery draws, the post-action posterior, numeric seed, absolute time and fixed formation identifiers are forbidden online inputs. A GNN is not the default: a calibrated monotone logistic model or shallow tree is tried first. A GNN is considered only if a repeatable interaction residual remains after the interpretable state is frozen.

## Paired teacher

Each teacher example starts from exactly the same pre-action state. The action and static-full continuation share measurements, delivery uniforms and filter RNG. The horizon equals one action page plus the directed formation-cycle diameter: four pages for M24 and six for X36. Labels retain separate network, formation, sensor, downstream-peer and terminal-consensus regrets; an action is unsafe if any registered local or network regret is positive.

The statistical unit is the complete `scene × seed` trajectory. Windows from one trajectory may not be randomly split across development, calibration and validation. Initial radial-scene seeds are registered in `getCounterfactualRegretGateV133Protocol`; richer convoy, relay, merge-split and curved-corridor scenes enter only after the radial causal gate is frozen.

## Gates

Before tracking evaluation, the conservative classifier must recall at least 90% of harmful actions, keep false-safe rate at or below 5%, act dynamically on at least 10% of eligible states and retain at least half of the paired oracle gain. The frozen tracking version must then deliver at least 5% mean and mature-stage improvement on both M24 and X36, keep every unseen seed positive, avoid worst-formation, worst-sensor and downstream-peer regression, and preserve net communication savings. Runtime and memory overhead are capped at 1.20× and 1.25× the static reference.

Failed candidates, parameter searches and diagnostic negative results remain repository experiment records. The main progress document receives only a gate-passing result or a repeated mechanism finding that materially changes the method.
