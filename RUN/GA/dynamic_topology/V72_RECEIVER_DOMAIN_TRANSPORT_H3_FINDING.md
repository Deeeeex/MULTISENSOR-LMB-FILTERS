# V72 receiver-domain transport H=3 finding

V72 falsifies the complete V70--V72 decision pipeline: its selected routes do
not have reliable positive three-step network value. The failure is not caused
by an infeasible graph, message sparsification, changed fusion weights, or a
material byte increase.
Both candidates preserve the reference message count and per-row positive
weight multiset, pass selected rolling-B3 connectivity, and change attempted
bytes by less than 0.2%.

| Scale | Mean E-OSPA | Worst sensor | Minimum formation | Window consensus | Terminal consensus | Attempted bytes | Strong gate |
|:--|--:|--:|--:|--:|--:|--:|:--:|
| M24 | +0.473% | +0.000% | +0.000% | -0.528% | -1.061% | -0.158% | fail |
| X36 | -0.290% | +0.000% | -2.399% | +0.142% | -0.031% | -0.013% | fail |

The formation-resolved outcome identifies a sharp proxy/outcome mismatch.
For M24, the directly affected formation 3 improves by 2.579%, but the
network-average gain is only 0.473% and consensus degrades. For X36,
formation 4 improves by 0.510% while formation 5 degrades by 2.399%. This is
an ordering inversion: formation 5 contributed substantially more V71 local
net mass than formation 4, yet produced the worse closed-loop outcome.

Post-run review exposes two unresolved causes. First, V68--V71 call
`fuseLmbPosteriorsByLabel` without the formal tracking configuration, so their
source score uses the moment-matched receiver. V72 tracking instead enables
mixture-aware heavy fusion. The selector and executor therefore do not yet
share receiver semantics. Second, even after that correction, a route
replacement changes the posterior at one receiver in the current KLA round
and may propagate through label-dependent fusion weights in later rounds. The
current score measures only the first endpoint; it omits downstream
attenuation or amplification, cancellation between receiver slots,
formation-tail exposure, and consensus debt. V72 does not yet isolate which
cause dominates.

The time-resolved V72 outcome separates the layers further. In M24, affected
formation 3 improves by 3.128 E-OSPA units in the intervention step, is almost
neutral one step later, and improves by 1.658 units in the final step, while
network consensus worsens at every step. In X36, formation 4 improves by
1.498 units immediately but becomes 0.754 units worse by the third step;
formation 5 is already 1.605 units worse in the intervention step and reaches
3.107 units worse by the third step. Therefore temporal propagation is not the
only missing term: the existence-only direct score fails to detect immediate
spatial/cardinality task risk for formation 5, while formation 4 separately
shows a later propagation or recovery effect.

The next method decision is ordered rather than simultaneous:

1. recompute direct, label-wise influence with the same mixture-aware
   configuration, receiver-first input ordering, and heavy-message semantics
   as the formal filter;
2. augment the aligned direct score with truth-free posterior Bayes risk and
   one-round network disagreement so spatial uncertainty and immediate
   consensus debt enter the decision;
3. only after the direct task-risk layer is adequate, add a source-only
   multi-round propagation term on the reference fusion graph, with explicit
   formation-tail and consensus penalties.

V71 normalization thresholds and formation budgets will not be tuned further.
The physical, message-count, row-weight, decision-retention, rolling-B3, and
deterministic-reference fallback constraints remain valid and will be reused.
A learned residual is justified only if receiver-aligned and propagation-aware
analytical proxies still leave a repeatable gap after cross-scale source-only
checks.

These two merge-split anchors are opened development evidence. They may be
used to diagnose and construct the V73 proxy, but not to claim validation.
Any frozen method must later move to the already qualified braided-handover
scenes and new seeds before a cross-scene generalization claim is allowed.
