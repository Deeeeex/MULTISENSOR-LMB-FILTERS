# Dynamic-topology oracle-gap screen

- Preset: `m24-hard`
- Seeds: `[7 17]`
- Generated: 2026-07-26 04:32:53
- Decision status: `insufficient-arms`

- Focus window: `teacher-handover-and-blockage`, steps `[75 77]`

- Analysis window: steps `[75 77]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Undirected edges | Directed routes | Attempts | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed reliability control (w=0.15) | 18.8933 | 36.7448 | 0.9989 | 22.7142 | 0.5208 | 3503940 | 3452068 | 20.00 | 24.00 | 72 | 0.4786 | 0.0000 | 0.04 | 29.93 |
| Directed reliability control (w=0.30) | 17.9975 | 30.8060 | 0.9371 | 21.5217 | 0.4722 | 3501756 | 3450220 | 20.00 | 24.00 | 72 | 0.4786 | 0.0000 | 0.04 | 30.22 |
| Directed reliability control (w=0.50) | 18.6913 | 33.0095 | 0.8980 | 22.0653 | 0.5278 | 3502428 | 3450892 | 20.00 | 24.00 | 72 | 0.4786 | 0.0000 | 0.04 | 31.61 |
| Directed reliability control (w=0.70) | 18.7077 | 32.8346 | 0.9086 | 22.0616 | 0.5417 | 3500160 | 3448624 | 20.00 | 24.00 | 72 | 0.4786 | 0.0000 | 0.04 | 31.35 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| Directed reliability control (w=0.15) | 18.8933 | 0.9989 | 22.7142 | 0.5208 |
| Directed reliability control (w=0.30) | 17.9975 | 0.9371 | 21.5217 | 0.4722 |
| Directed reliability control (w=0.50) | 18.6913 | 0.8980 | 22.0653 | 0.5278 |
| Directed reliability control (w=0.70) | 18.7077 | 0.9086 | 22.0616 | 0.5417 |

## Registered gate readout

- Constraint-eligible arms: `0`
- Best observed arm: ``
- Best observed focus E-OSPA: NaN
- Best observed gain vs static: NaN%
- Minimum practical tracking gain: 5.00%
- Best observed byte mismatch vs static: NaN%
- Oracle consensus gain: NaN%
- Oracle tracking gain: NaN%
- Analytic share of static-to-oracle gain: NaN
- Diagnostic reference dominated: `0`
- Attempted-byte mismatch: NaN%
- Recommendation: Run the complete registered arm set.

## Learned directed-routing readout

- Eligible learned directed arms: `0`
- Best learned directed arm: ``
- Focus E-OSPA: NaN
- Gain vs static: NaN%
- Gain vs local: NaN%
- Worst-node gain vs static: NaN%
- Worst-node gain vs local: NaN%
- Attempted bytes relative to static: NaN%
- Mean selected directed routes: NaN
- Zero infeasibility: `0`
- Strict tail-safe vs static: `0`
- Strict tail-safe vs local: `0`
- Passes registered mean-tracking gate: `0`

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
- Sparse directed routing is evaluated as a Pareto arm: it must use no more attempted payload bytes than static (within 2% accounting tolerance) and must beat both static and local on mean tracking. It is not required to match the static edge count.
- This is a conditional continuation experiment. It compares policy decisions after a common static prefix and does not estimate full-episode performance.
- The static arm is selected by all-time geometry and link distance; it is not an exhaustive offline performance optimum.
- Exact one-step action enumeration is not a closed-loop upper bound. If it is dominated by a deployable arm, it cannot justify a learned teacher or an analytic-sufficiency claim.
