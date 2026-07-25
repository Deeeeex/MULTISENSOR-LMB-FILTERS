# Dynamic-topology oracle-gap screen

- Preset: `m24-hard`
- Seeds: `7`
- Generated: 2026-07-26 03:43:00
- Decision status: `directed-routing-screening-gain`

- Focus window: `teacher-handover-and-blockage`, steps `[75 77]`

- Analysis window: steps `[75 77]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Undirected edges | Directed routes | Attempts | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Local only | 23.4600 | 42.0469 | 1.0402 | 25.1022 | 0.7500 | 0 | 0 | 29.00 | 58.00 | 0 | 0.0000 | 0.0000 | 0.00 | 16.66 |
| All-time geometry static | 25.3731 | 42.6223 | 0.7952 | 26.0571 | 0.9028 | 8494176 | 8041800 | 29.00 | 58.00 | 174 | 0.0000 | 0.0000 | 0.00 | 34.33 |
| M24-trained kNN directed routing | 15.3711 | 34.6887 | 0.9523 | 20.0986 | 0.3472 | 2626840 | 2272808 | 15.67 | 18.67 | 56 | 0.8993 | 0.0000 | 16.00 | 39.00 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| Local only | 23.4600 | 1.0402 | 25.1022 | 0.7500 |
| All-time geometry static | 25.3731 | 0.7952 | 26.0571 | 0.9028 |
| M24-trained kNN directed routing | 15.3711 | 0.9523 | 20.0986 | 0.3472 |

## Registered gate readout

- Constraint-eligible arms: `1`
- Best observed arm: `All-time geometry static`
- Best observed focus E-OSPA: 25.3731
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
- Focus E-OSPA: 15.3711
- Gain vs static: 39.42%
- Gain vs local: 34.48%
- Worst-node gain vs static: 18.61%
- Worst-node gain vs local: 17.50%
- Attempted bytes relative to static: 30.93%
- Mean selected directed routes: 18.67
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
