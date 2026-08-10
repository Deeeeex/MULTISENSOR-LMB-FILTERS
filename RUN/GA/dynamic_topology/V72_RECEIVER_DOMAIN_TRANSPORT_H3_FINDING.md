# V72 receiver-domain transport H=3 finding

V72 falsifies the decision rule used by V70/V71: summing positive
receiver-domain net mass is not sufficient to select a route with positive
three-step network value. The failure is not caused by an infeasible graph,
message sparsification, changed fusion weights, or a material byte increase.
Both candidates preserve the reference message count and per-row positive
weight multiset, pass selected rolling-B3 connectivity, and change attempted
bytes by less than 0.2%.

| Scale | Mean E-OSPA | Worst sensor | Minimum formation | Window consensus | Terminal consensus | Attempted bytes | Strong gate |
|:--|--:|--:|--:|--:|--:|--:|:--:|
| M24 | +0.473% | +0.000% | +0.000% | -0.528% | -1.061% | -0.158% | fail |
| X36 | -0.290% | +0.000% | -2.399% | +0.142% | -0.031% | -0.013% | fail |

The formation-resolved outcome identifies the missing term more sharply.
For M24, the directly affected formation 3 improves by 2.579%, but the
network-average gain is only 0.473% and consensus degrades. For X36,
formation 4 improves by 0.510% while formation 5 degrades by 2.399%. This is
an ordering inversion: formation 5 contributed substantially more V71 local
net mass than formation 4, yet produced the worse closed-loop outcome.

The first-principles explanation is temporal propagation. A route replacement
changes the posterior at a receiver in the current KLA round. In later rounds
that receiver becomes an input to other receivers, so the effect propagates
through label-dependent fusion weights and overlap. A direct receiver-domain
score measures only the first endpoint; it omits downstream attenuation or
amplification, cancellation between receiver slots, formation-tail exposure,
and the consensus debt induced by spatially uneven changes.

The next method must therefore separate two terms:

1. a direct, label-wise KLA decision influence at every changed receiver;
2. a source-only multi-round propagation term computed on the reference
   fusion graph, with explicit formation-tail and consensus penalties.

V71 normalization thresholds and formation budgets will not be tuned further.
The physical, message-count, row-weight, decision-retention, rolling-B3, and
deterministic-reference fallback constraints remain valid and will be reused.
A learned residual is justified only if the propagation-aware analytical
proxy still leaves a repeatable gap after cross-scale source-only checks.

These two merge-split anchors are opened development evidence. They may be
used to diagnose and construct the V73 proxy, but not to claim validation.
Any frozen method must later move to the already qualified braided-handover
scenes and new seeds before a cross-scene generalization claim is allowed.
