# Dynamic-topology oracle-gap screen

- Preset: `x36-clean-scale`
- Seeds: `7`
- Generated: 2026-07-26 07:06:25
- Decision status: `insufficient-arms`

- Focus window: `clean-scale-handover-and-blockage`, steps `[75 77]`

- Analysis window: steps `[75 77]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Undirected edges | Directed routes | Attempts | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed reliability control (w=0.50) | 37.9153 | 50.7212 | 1.1524 | 28.6726 | 1.6019 | 8290440 | 8053296 | 30.00 | 36.00 | 108 | 0.4859 | 0.0000 | 0.07 | 87.98 |
| Directed moment compatibility control (w=0.50) | 40.4920 | 63.6772 | 1.1791 | 33.2933 | 1.8796 | 8184648 | 8033336 | 23.67 | 36.00 | 108 | 0.4327 | 0.0000 | 65.08 | 154.61 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| Directed reliability control (w=0.50) | 37.9153 | 1.1524 | 28.6726 | 1.6019 |
| Directed moment compatibility control (w=0.50) | 40.4920 | 1.1791 | 33.2933 | 1.8796 |

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
