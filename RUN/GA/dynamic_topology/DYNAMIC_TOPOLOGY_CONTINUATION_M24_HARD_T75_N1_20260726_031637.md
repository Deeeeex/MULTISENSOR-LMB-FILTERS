# Dynamic-topology oracle-gap screen

- Preset: `m24-hard`
- Seeds: `7`
- Generated: 2026-07-26 03:16:37
- Decision status: `directed-routing-screening-gain-tail-caveat`

- Focus window: `teacher-handover-and-blockage`, steps `[75 75]`

- Analysis window: steps `[75 75]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Undirected edges | Directed routes | Attempts | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Local only | 24.5451 | 42.9270 | 0.9747 | 26.4274 | 0.8333 | 0 | 0 | 29.00 | 58.00 | 0 | 0.0000 | 0.0000 | 0.00 | 6.02 |
| All-time geometry static | 25.5087 | 42.6616 | 0.8090 | 27.9094 | 0.9167 | 2808536 | 2611128 | 29.00 | 58.00 | 58 | 0.0000 | 0.0000 | 0.00 | 8.72 |
| M24-trained kNN directed routing | 15.4348 | 42.7555 | 0.9257 | 20.8647 | 0.3333 | 1158960 | 910688 | 23.00 | 24.00 | 24 | 0.9067 | 0.0000 | 5.68 | 12.14 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| Local only | 24.5451 | 0.9747 | 26.4274 | 0.8333 |
| All-time geometry static | 25.5087 | 0.8090 | 27.9094 | 0.9167 |
| M24-trained kNN directed routing | 15.4348 | 0.9257 | 20.8647 | 0.3333 |

## Registered gate readout

- Constraint-eligible arms: `1`
- Best observed arm: `All-time geometry static`
- Best observed focus E-OSPA: 25.5087
- Best observed gain vs static: 0.00%
- Minimum practical tracking gain: 5.00%
- Best observed byte mismatch vs static: 0.00%
- Oracle consensus gain: NaN%
- Oracle tracking gain: NaN%
- Analytic share of static-to-oracle gain: NaN
- Diagnostic reference dominated: `0`
- Attempted-byte mismatch: NaN%
- Recommendation: The deployment-observable directed-routing arm passes the mean-tracking and communication screen, but its strict worst-node comparison against static still needs validation.

## Learned directed-routing readout

- Eligible learned directed arms: `1`
- Best learned directed arm: `M24-trained kNN directed routing`
- Focus E-OSPA: 15.4348
- Gain vs static: 39.49%
- Gain vs local: 37.12%
- Worst-node gain vs static: -0.22%
- Worst-node gain vs local: 0.40%
- Attempted bytes relative to static: 41.27%
- Mean selected directed routes: 24.00
- Zero infeasibility: `1`
- Strict tail-safe vs static: `0`
- Strict tail-safe vs local: `1`
- Passes registered mean-tracking gate: `1`

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
- Sparse directed routing is evaluated as a Pareto arm: it must use no more attempted payload bytes than static (within 2% accounting tolerance) and must beat both static and local on mean tracking. It is not required to match the static edge count.
- This is a conditional continuation experiment. It compares policy decisions after a common static prefix and does not estimate full-episode performance.
- The static arm is selected by all-time geometry and link distance; it is not an exhaustive offline performance optimum.
- Exact one-step action enumeration is not a closed-loop upper bound. If it is dominated by a deployable arm, it cannot justify a learned teacher or an analytic-sufficiency claim.
