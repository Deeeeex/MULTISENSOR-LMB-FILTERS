# ICASSP 2027 Storyline

## Working title

Light Posterior Exchange for Communication-Efficient Distributed LMB Fusion

## One-sentence claim

For single-round distributed GA/KLA-LMB fusion, preserving the effective fusion graph while replacing full Gaussian-mixture LMB messages with label-wise moment-matched light posteriors can cut communication by about 58% without measurable tracking or consensus loss in the validated 4+4 multisensor formation benchmark.

## Core story

The original event-triggered branch showed that saving bytes by dropping messages is fragile: sparse triggering and dynamic topology can break the effective information-flow graph that KLA fusion needs, even when the nominal communication graph remains connected. The later held-out experiments point to a simpler and cleaner contribution. The receiver-side fusion path already consumes label-wise moment-matched Bernoulli statistics, so transmitting every Gaussian-mixture component is often unnecessary. A periodic light-posterior backbone keeps the KLA graph intact and compresses the payload instead of sparsifying the graph.

The paper should therefore frame the method as payload compression under graph preservation, not as a final dual-threshold event-triggered policy. Dynamic topology, mixed payload, and event-triggered variants are useful as diagnostics and ablations, but the main positive result is the static periodic light-posterior exchange.

## Evidence to foreground

- Held-out 50-trial result, seeds 32-81, 100 steps.
- Periodic full posterior baseline: 27,771,195 estimated bytes, local E-OSPA 2.0652, consensus OSPA 1.9501, effective-weight lambda2 0.373.
- Periodic light posterior on static topology: 11,503,339 estimated bytes, 58.6% byte reduction, unchanged local E-OSPA, consensus OSPA, position disagreement, cardinality dispersion, and effective-weight lambda2 0.373; 50/50 pass rate.
- Periodic full posterior plus dynamic topology increases communication by 6.1% and degrades consensus OSPA by 11.1%, showing topology adaptation alone is not the answer.
- Periodic light posterior plus guarded dynamic topology still reduces bytes by 58.3%, but pass rate drops to 35/50, so graph-preserving static light exchange is the safer mainline.

## Claims to avoid

- Do not claim the final method is event-triggered sparse communication.
- Do not claim dynamic topology is the primary contribution.
- Do not claim full GM-LMB heavy messages improve the current single-round fusion result.
- Do not claim mixed label-wise payload is necessary for the current result; treat it as a later compression ablation if used.

## Short-paper structure

1. Introduction: communication bottleneck; why graph sparsification is risky; payload compression is the safer lever.
2. Related work: Labeled RFS/LMB tracking, conservative KLA/GCI fusion, communication-aware RFS fusion.
3. Method: full posterior exchange baseline, light posterior message, graph-preserving exchange protocol, diagnostics for effective KLA graph.
4. Experiments: 4+4 benchmark, N50 held-out table, ablation interpretation.
5. Conclusion: full posterior exchange can be overkill when the fusion path moment-matches labels; preserve information flow first, compress payload second.
