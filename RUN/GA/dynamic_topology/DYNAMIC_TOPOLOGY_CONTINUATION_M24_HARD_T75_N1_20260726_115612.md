# Dynamic-topology oracle-gap screen

- Preset: `m24-hard`
- Seeds: `7`
- Generated: 2026-07-26 11:56:12
- Decision status: `dynamic-attribution-incomplete-controls`

- Focus window: `teacher-handover-and-blockage`, steps `[75 77]`

- Analysis window: steps `[75 77]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Undirected edges | Directed routes | Attempts | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed fixed index control (w=0.30) | 19.9772 | 30.7298 | 0.9299 | 22.1367 | 0.5694 | 3501360 | 3448480 | 20.00 | 24.00 | 72 | 0.4786 | 0.0000 | 0.00 | 26.16 |
| Strong formation cycle v3 (bridge=0.05, w=0.30) | 19.6678 | 35.6043 | 0.9524 | 22.8852 | 0.5694 | 3530832 | 3429776 | 20.67 | 24.00 | 72 | 0.6202 | 0.0000 | 17.07 | 43.13 |
| Directed fixed index control (w=0.50) | 20.0187 | 35.1414 | 0.8766 | 21.7539 | 0.6111 | 3505392 | 3452512 | 20.00 | 24.00 | 72 | 0.4786 | 0.0000 | 0.00 | 26.19 |
| Strong formation cycle v3 (bridge=0.05, w=0.50) | 20.1208 | 35.2231 | 0.9088 | 23.2643 | 0.6250 | 3513336 | 3412280 | 20.33 | 24.00 | 72 | 0.5645 | 0.0000 | 17.04 | 42.99 |
| Directed fixed index control (w=0.70) | 19.2470 | 34.6420 | 0.8840 | 21.0525 | 0.5833 | 3512112 | 3459232 | 20.00 | 24.00 | 72 | 0.4786 | 0.0000 | 0.00 | 26.55 |
| Strong formation cycle v3 (bridge=0.05, w=0.70) | 18.5724 | 34.6420 | 0.9261 | 20.8730 | 0.5278 | 3507576 | 3406520 | 20.00 | 24.00 | 72 | 0.5760 | 0.0000 | 17.13 | 43.15 |

## Analysis-window route-attribution diagnostics

| Arm | Boundary-inclusive churn | Prefix receiver changes | Within-window receiver changes | Distinct maps | Different from fixed index | Receiver coverage | Unique senders / receiver | Cross-formation routes |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed fixed index control (w=0.30) | 0.4786 | 1.0000 | 0.0000 | 1.00 | 0.0000 | 1.0000 | 1.00 | 0.0000 |
| Strong formation cycle v3 (bridge=0.05, w=0.30) | 0.6202 | 1.0000 | 0.2292 | 3.00 | 0.1667 | 1.0000 | 1.42 | 0.1667 |
| Directed fixed index control (w=0.50) | 0.4786 | 1.0000 | 0.0000 | 1.00 | 0.0000 | 1.0000 | 1.00 | 0.0000 |
| Strong formation cycle v3 (bridge=0.05, w=0.50) | 0.5645 | 1.0000 | 0.1250 | 3.00 | 0.1667 | 1.0000 | 1.25 | 0.1667 |
| Directed fixed index control (w=0.70) | 0.4786 | 1.0000 | 0.0000 | 1.00 | 0.0000 | 1.0000 | 1.00 | 0.0000 |
| Strong formation cycle v3 (bridge=0.05, w=0.70) | 0.5760 | 1.0000 | 0.1667 | 3.00 | 0.1667 | 1.0000 | 1.33 | 0.1667 |

When continuation is used, boundary-inclusive churn also contains the one-time transition from the common static prefix. Only the within-window receiver-change rate and distinct-map count establish that an arm actually changes routes during the evaluated window.

## Gateway and effective-graph diagnostics

| Arm | Gateway-only changes | Gateway maps | Instant weak | Instant strong | Union weak | Union strong | Formation coverage |
|:--|--:|--:|--:|--:|--:|--:|--:|
| Directed fixed index control (w=0.30) | 0.0000 | 1.00 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| Strong formation cycle v3 (bridge=0.05, w=0.30) | 0.2292 | 3.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Directed fixed index control (w=0.50) | 0.0000 | 1.00 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| Strong formation cycle v3 (bridge=0.05, w=0.50) | 0.1250 | 3.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Directed fixed index control (w=0.70) | 0.0000 | 1.00 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| Strong formation cycle v3 (bridge=0.05, w=0.70) | 0.1667 | 3.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |

Gateway-only metrics mask out the synchronized intra-formation round-robin backbone. Union connectivity is computed on the directed formation graph over the complete focus window; the attribution gate requires at least weak connectivity.

## Focus-window result

| Arm | Focus E-OSPA | Focus worst node | Focus attempted bytes | Focus route changes | Focus maps | Focus coverage | Different from fixed index | Focus cross-formation | Focus infeasible | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed fixed index control (w=0.30) | 19.9772 | 30.7298 | 3501360 | 0.0000 | 1.00 | 1.0000 | 0.0000 | 0.0000 | 0.0000 | 0.9299 | 22.1367 | 0.5694 |
| Strong formation cycle v3 (bridge=0.05, w=0.30) | 19.6678 | 35.6043 | 3530832 | 0.2292 | 3.00 | 1.0000 | 0.1667 | 0.1667 | 0.0000 | 0.9524 | 22.8852 | 0.5694 |
| Directed fixed index control (w=0.50) | 20.0187 | 35.1414 | 3505392 | 0.0000 | 1.00 | 1.0000 | 0.0000 | 0.0000 | 0.0000 | 0.8766 | 21.7539 | 0.6111 |
| Strong formation cycle v3 (bridge=0.05, w=0.50) | 20.1208 | 35.2231 | 3513336 | 0.1250 | 3.00 | 1.0000 | 0.1667 | 0.1667 | 0.0000 | 0.9088 | 23.2643 | 0.6250 |
| Directed fixed index control (w=0.70) | 19.2470 | 34.6420 | 3512112 | 0.0000 | 1.00 | 1.0000 | 0.0000 | 0.0000 | 0.0000 | 0.8840 | 21.0525 | 0.5833 |
| Strong formation cycle v3 (bridge=0.05, w=0.70) | 18.5724 | 34.6420 | 3507576 | 0.1667 | 3.00 | 1.0000 | 0.1667 | 0.1667 | 0.0000 | 0.9261 | 20.8730 | 0.5278 |

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
- Recommendation: Run the candidate-specific registered controls at the same source weight and action support before attributing online gain. Gateway candidates require round-robin, fixed, rotating and link-aware gateway controls.

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

- Eligible residual arms: `0`
- Residual arm: ``
- Registered backbone: ``
- Residual focus E-OSPA: NaN
- Backbone focus E-OSPA: NaN
- Gain vs backbone: NaN%
- Worst-node gain vs backbone: NaN%
- Attempted bytes relative to backbone: NaN%
- Policy-time overhead: NaN s
- Mean learned override fraction: NaN
- Mean in-support candidate fraction: NaN
- Exact backbone match: `0`
- Positive gain on every seed: `0`
- Strict tail-safe vs backbone: `0`
- Within backbone attempted bytes (+2%): `0`
- Minimum practical incremental gain: NaN%
- Passes incremental-learning gate: `0`
- Residual-routing status: `unavailable`

## Dynamic-routing attribution readout

- Candidate arm: `Strong formation cycle v3 (bridge=0.05, w=0.70)`
- Available candidates: `3`
- Candidates with complete matched controls: `0`
- Structurally eligible candidates: `0`
- Strong-control reference: ``
- Weight-matched reference: ``
- Complete registered control set: `0`
- Candidate source weight: 0.70
- Weight-matched control set: `0`
- Matched fixed-gateway phases: `0`
- Matched rotating-gateway phases: `0`
- Required gateway phases per family: `6`
- Candidate uses cross-formation routes: `1`
- Cross-formation scheduled control set: `0`
- Action support matched: `0`
- Gain vs weight-matched control: NaN%
- Gain vs weight-matched control by seed: `[]`
- Positive matched-control gain on every seed: `0`
- Gain vs strongest control: NaN%
- Gain by seed: `[]`
- Positive gain on every seed: `0`
- Strict tail-safe vs strongest control: `0`
- Maximum attempted-byte mismatch: NaN%
- Attempted bytes matched within 2%: `0`
- No more attempted bytes on every seed: `0`
- Strict tracking-byte Pareto on every seed: `0`
- Passes communication fairness: `0`
- Distinct maps on every seed: `0`
- Within-window changes on every seed: `0`
- Distinct gateway-only maps on every seed: `0`
- Gateway-only changes on every seed: `0`
- Formation-union weak connectivity every seed: `0`
- Formation-union strong connectivity every seed: `0`
- Minimum instantaneous weak-connectivity fraction: NaN
- Minimum gateway formation coverage: NaN
- Minimum difference from fixed index: NaN
- Complete receiver coverage: `0`
- Cross-formation routes observed: `0`
- Passes dynamic-attribution gate: `0`
- Dynamic-attribution status: `dynamic-attribution-incomplete-controls`

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
- Sparse directed routing is evaluated as a Pareto arm: it must use no more attempted payload bytes than static (within 2% accounting tolerance) and must beat both static and local on mean tracking. It is not required to match the static edge count.
- This is a conditional continuation experiment. It compares policy decisions after a common static prefix and does not estimate full-episode performance.
- The static arm is selected by all-time geometry and link distance; it is not an exhaustive offline performance optimum.
- Exact one-step action enumeration is not a closed-loop upper bound. If it is dominated by a deployable arm, it cannot justify a learned teacher or an analytic-sufficiency claim.
