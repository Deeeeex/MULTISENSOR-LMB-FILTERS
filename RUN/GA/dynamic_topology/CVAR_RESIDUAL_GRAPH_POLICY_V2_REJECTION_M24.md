# M24 innovation-aware CVaR graph policy v2: rejection note

## Evidence boundary

This is a training-only, whole-seed LOSO result on seeds
`[11, 17, 19, 23, 27, 29]` and times `78:83`. Development seeds 31 and
37 remained sealed. The 36 v2 states passed the independent feature,
predecessor-target provenance, topology, and safety-label audit before
training.

## Result

| Model | Tail recall | Worst-seed recall | Edge task rho | Median regret | P90 regret | False-safe | Safe recall |
|:--|--:|--:|--:|--:|--:|--:|--:|
| Ridge sentinel | 0.157 | 0.111 | 0.339 | 0.351 | 0.530 | 0.748 | 0.040 |
| Relational MLP | 0.167 | 0.000 | 0.401 | 0.372 | 0.575 | 0.301 | 0.018 |

The selected true task advantage was negative on five of six held-out
seeds for both model families. Neither family passed the node-risk,
edge-task, regret, safety, or task-advantage gate. Development
evaluation, H=3 return generation, held-out M24, and X36 therefore remain
unauthorized.

## Finding

The v2 feature replay was causal and nontrivial: every state contained
fresh three-step local update diagnostics, and the old 191-dimensional
feature prefix matched the audited v1 shards exactly. The failure is
therefore not explained by stale continuation defaults, target drift, or
teacher recomputation.

The added innovation, association-confidence, and NIS summaries did not
make per-node top-3 truth risk predictable across seeds. More
importantly, this node-level target is misaligned with the actual action:
the controller replaces four bridges in a formation-level residual
cycle. Tail nodes also frequently cluster within one formation. A small
node-ranking error is amplified by the discrete cycle projection, which
explains why moderate edge-score correlation still produces negative
cycle-level advantage.

## Authorized pivot

Do not increase node-MLP capacity or open development labels. The next
analysis must decompose node-risk and edge-value errors and test an
action-aligned abstraction:

1. aggregate causal node evidence into formation-level risk;
2. rank candidate bridges within directed formation pairs;
3. select a formation cycle with the exact message-count and dominant
   backbone invariants;
4. calibrate safety at the formation/action level rather than requiring
   exact top-3 node recovery.

Only a training-seed LOSO gain in structured action regret can authorize
one frozen development evaluation.
