# Dynamic-topology oracle-gap screen

- Preset: `x36-clean-scale`
- Seeds: `7`
- Generated: 2026-07-26 03:22:38
- Decision status: `directed-routing-screening-gain`

- Focus window: `clean-scale-handover-and-blockage`, steps `[75 75]`

- Analysis window: steps `[75 75]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Undirected edges | Directed routes | Attempts | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Local only | 44.4497 | 61.7640 | 1.1016 | 33.9585 | 2.1667 | 0 | 0 | 44.00 | 88.00 | 0 | 0.0000 | 0.0000 | 0.00 | 21.65 |
| All-time geometry static | 46.8048 | 68.6548 | 0.9986 | 36.8924 | 2.4444 | 6648128 | 6494320 | 44.00 | 88.00 | 88 | 0.0000 | 0.0000 | 0.00 | 29.75 |
| M24-trained kNN directed routing | 41.6002 | 61.7640 | 1.1311 | 33.1179 | 1.9722 | 1385064 | 1304440 | 16.00 | 18.00 | 18 | 0.8723 | 0.0000 | 21.54 | 43.82 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| Local only | 44.4497 | 1.1016 | 33.9585 | 2.1667 |
| All-time geometry static | 46.8048 | 0.9986 | 36.8924 | 2.4444 |
| M24-trained kNN directed routing | 41.6002 | 1.1311 | 33.1179 | 1.9722 |

## Registered gate readout

- Constraint-eligible arms: `1`
- Best observed arm: `All-time geometry static`
- Best observed focus E-OSPA: 46.8048
- Best observed gain vs static: 0.00%
- Minimum practical tracking gain: 5.00%
- Best observed byte mismatch vs static: 0.00%
- Oracle consensus gain: NaN%
- Oracle tracking gain: NaN%
- Analytic share of static-to-oracle gain: NaN
- Diagnostic reference dominated: `0`
- Attempted-byte mismatch: NaN%
- Recommendation: The deployment-observable directed-routing arm passes the bounded mean-tracking, tail and communication screen; run multi-step and multi-seed validation before a paper claim.

## Learned directed-routing readout

- Eligible learned directed arms: `1`
- Best learned directed arm: `M24-trained kNN directed routing`
- Focus E-OSPA: 41.6002
- Gain vs static: 11.12%
- Gain vs local: 6.41%
- Worst-node gain vs static: 10.04%
- Worst-node gain vs local: 0.00%
- Attempted bytes relative to static: 20.83%
- Mean selected directed routes: 18.00
- Zero infeasibility: `1`
- Strict tail-safe vs static: `1`
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
