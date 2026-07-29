# M24 observable-reward route decision

The current dynamic-topology design keeps the fixed dominant backbone and
the message-count-preserving residual-cycle projector. This protects the
nominal rolling information-flow condition and the attempted-byte budget.
The unresolved component was how to choose the residual edges without
target truth.

Three increasingly structured observable-feedback tests have now
completed on the same already-opened M24 training split
`[11 17 19 23 27 29]`, times `78:83`.

| Test | Main evidence | Result | Authorization |
|:--|:--|:--|:--|
| Direct Poisson-intensity reward | global Spearman 0.4196; pairwise 0.5903; teacher-positive 0.5417 | absolute and pairwise gates failed | rejected |
| Cardinality-matched IID-cluster reward | global Spearman 0.4384; pairwise 0.6111; teacher-positive 0.5347 | ranking improved, absolute gate failed | rejected |
| Nested whole-seed ordinal learnability | surrogate pairwise 0.5069; surrogate Spearman 0.0479; positive-seed fraction 2/6 | observable ranking did not transfer across seeds | rejected |

The complete convex grid
\((1-\alpha)S_{\mathrm{Poisson}}+\alpha S_{\mathrm{IID}}\),
\(\alpha=0:0.01:1\), contained no score passing all frozen gates.
Therefore the failure is not repaired by tuning the cardinality weight.

The important distinction is:

1. the IID-cluster score contains real within-state task-ranking signal;
2. that signal is not represented stably by the current 47-dimensional
   aggregate edge features, even after adding receiver and sender node
   summaries;
3. therefore generating more labels of the same feature/target type is
   not evidence-backed.

This closes the direct reward-bandit, score-composite, and aggregate
feature ordinal-ranker branches. It does not falsify safe dynamic topology
itself, because the privileged H3 teacher still demonstrates attainable
M24 headroom under the same projector. It instead localizes the bottleneck
to state representation and policy learning.

The next admissible method redesign is simulator-trained safe graph
control using a label-set representation:

- encode per-label existence, covariance, association ambiguity, and
  sender-receiver disagreement before pooling;
- use a permutation-equivariant set encoder followed by edge message
  passing;
- train end-to-end on closed-loop tracking return in simulation rather
  than imitate a one-step teacher or regress a proxy score;
- retain the deterministic residual-cycle projector as the hard
  connectivity/message-count layer;
- freeze the representation and policy on training seeds before opening
  any new M24 validation seed; X36 remains locked until M24 passes.

No bandit, expanded surrogate dataset, development evaluation, held-out
M24, or X36 run is authorized by the present evidence.
