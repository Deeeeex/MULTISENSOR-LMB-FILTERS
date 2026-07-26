# Dynamic-topology oracle-gap screen

- Preset: `m24-hard`
- Seeds: `[7 17]`
- Generated: 2026-07-26 07:59:07
- Decision status: `residual-routing-no-incremental-gain`

- Focus window: `teacher-handover-and-blockage`, steps `[75 77]`

- Analysis window: steps `[75 77]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Undirected edges | Directed routes | Attempts | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed reliability control (w=0.50) | 18.6913 | 33.0095 | 0.8980 | 22.0653 | 0.5278 | 3502428 | 3450892 | 20.00 | 24.00 | 72 | 0.4786 | 0.0000 | 0.03 | 26.36 |
| Support-gated residual directed KLA | 18.6913 | 33.0095 | 0.8980 | 22.0653 | 0.5278 | 3502428 | 3450892 | 20.00 | 24.00 | 72 | 0.4786 | 0.0000 | 17.11 | 43.28 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| Directed reliability control (w=0.50) | 18.6913 | 0.8980 | 22.0653 | 0.5278 |
| Support-gated residual directed KLA | 18.6913 | 0.8980 | 22.0653 | 0.5278 |

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
- Recommendation: The residual learner does not pass the registered practical gain, per-seed, tail, feasibility and communication gates against reliability-w0.50; do not attribute the backbone gain to learning.

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
- Directed-routing status: `unavailable`

## Residual-versus-backbone readout

- Eligible residual arms: `1`
- Residual arm: `Support-gated residual directed KLA`
- Registered backbone: `Directed reliability control (w=0.50)`
- Residual focus E-OSPA: 18.6913
- Backbone focus E-OSPA: 18.6913
- Gain vs backbone: 0.0000%
- Worst-node gain vs backbone: 0.0000%
- Attempted bytes relative to backbone: 100.00%
- Policy-time overhead: 17.074 s
- Mean learned override fraction: 0.0000
- Mean in-support candidate fraction: 0.9857
- Exact backbone match: `1`
- Positive gain on every seed: `0`
- Strict tail-safe vs backbone: `1`
- Within backbone attempted bytes (+2%): `1`
- Minimum practical incremental gain: 5.00%
- Passes incremental-learning gate: `0`
- Residual-routing status: `residual-routing-no-incremental-gain`

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
- Sparse directed routing is evaluated as a Pareto arm: it must use no more attempted payload bytes than static (within 2% accounting tolerance) and must beat both static and local on mean tracking. It is not required to match the static edge count.
- This is a conditional continuation experiment. It compares policy decisions after a common static prefix and does not estimate full-episode performance.
- The static arm is selected by all-time geometry and link distance; it is not an exhaustive offline performance optimum.
- Exact one-step action enumeration is not a closed-loop upper bound. If it is dominated by a deployable arm, it cannot justify a learned teacher or an analytic-sufficiency claim.
