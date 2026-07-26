# Dynamic-topology oracle-gap screen

- Preset: `x36-clean-scale`
- Seeds: `7`
- Generated: 2026-07-26 11:42:57
- Decision status: `dynamic-attribution-incomplete-controls`

- Focus window: `clean-scale-handover-and-blockage`, steps `[75 77]`

- Analysis window: steps `[75 77]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Undirected edges | Directed routes | Attempts | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed round robin control (w=0.40) | 44.1161 | 61.2646 | 1.1058 | 34.2052 | 2.1667 | 8155248 | 7996016 | 36.00 | 36.00 | 108 | 0.8448 | 0.0000 | 0.00 | 86.80 |
| Connected formation tree v2 (bridge=0.05, w=0.40) | 42.5002 | 56.3264 | 1.1202 | 34.0732 | 2.0185 | 8074728 | 7624608 | 36.00 | 36.00 | 108 | 0.8596 | 0.0000 | 79.35 | 164.99 |

## Analysis-window route-attribution diagnostics

| Arm | Boundary-inclusive churn | Prefix receiver changes | Within-window receiver changes | Distinct maps | Different from fixed index | Receiver coverage | Unique senders / receiver | Cross-formation routes |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed round robin control (w=0.40) | 0.8448 | 1.0000 | 1.0000 | 3.00 | 0.7778 | 1.0000 | 3.00 | 0.0000 |
| Connected formation tree v2 (bridge=0.05, w=0.40) | 0.8596 | 1.0000 | 0.9861 | 3.00 | 0.7963 | 1.0000 | 2.97 | 0.1389 |

When continuation is used, boundary-inclusive churn also contains the one-time transition from the common static prefix. Only the within-window receiver-change rate and distinct-map count establish that an arm actually changes routes during the evaluated window.

## Gateway and effective-graph diagnostics

| Arm | Gateway-only changes | Gateway maps | Instant weak | Instant strong | Union weak | Union strong | Formation coverage |
|:--|--:|--:|--:|--:|--:|--:|--:|
| Directed round robin control (w=0.40) | 0.0000 | 1.00 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| Connected formation tree v2 (bridge=0.05, w=0.40) | 0.2361 | 3.00 | 1.0000 | 0.0000 | 1.0000 | 0.0000 | 1.0000 |

Gateway-only metrics mask out the synchronized intra-formation round-robin backbone. Union connectivity is computed on the directed formation graph over the complete focus window; the attribution gate requires at least weak connectivity.

## Focus-window result

| Arm | Focus E-OSPA | Focus worst node | Focus attempted bytes | Focus route changes | Focus maps | Focus coverage | Different from fixed index | Focus cross-formation | Focus infeasible | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed round robin control (w=0.40) | 44.1161 | 61.2646 | 8155248 | 1.0000 | 3.00 | 1.0000 | 0.7778 | 0.0000 | 0.0000 | 1.1058 | 34.2052 | 2.1667 |
| Connected formation tree v2 (bridge=0.05, w=0.40) | 42.5002 | 56.3264 | 8074728 | 0.9861 | 3.00 | 1.0000 | 0.7963 | 0.1389 | 0.0000 | 1.1202 | 34.0732 | 2.0185 |

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

- Candidate arm: `Connected formation tree v2 (bridge=0.05, w=0.40)`
- Available candidates: `1`
- Candidates with complete matched controls: `0`
- Structurally eligible candidates: `0`
- Strong-control reference: `best per-seed registered fixed/scheduled control across weights`
- Weight-matched reference: `best per-seed weight-matched round-robin/fixed/rotating/link-aware gateway control`
- Complete registered control set: `0`
- Candidate source weight: 0.40
- Weight-matched control set: `0`
- Matched fixed-gateway phases: `0`
- Matched rotating-gateway phases: `0`
- Required gateway phases per family: `6`
- Candidate uses cross-formation routes: `1`
- Cross-formation scheduled control set: `0`
- Action support matched: `0`
- Gain vs weight-matched control: 3.6629%
- Gain vs weight-matched control by seed: `3.6629`
- Positive matched-control gain on every seed: `1`
- Gain vs strongest control: 3.6629%
- Gain by seed: `3.6629`
- Positive gain on every seed: `1`
- Strict tail-safe vs strongest control: `1`
- Maximum attempted-byte mismatch: 0.9873%
- Attempted bytes matched within 2%: `1`
- No more attempted bytes on every seed: `1`
- Strict tracking-byte Pareto on every seed: `1`
- Passes communication fairness: `1`
- Distinct maps on every seed: `1`
- Within-window changes on every seed: `1`
- Distinct gateway-only maps on every seed: `1`
- Gateway-only changes on every seed: `1`
- Formation-union weak connectivity every seed: `1`
- Formation-union strong connectivity every seed: `0`
- Minimum instantaneous weak-connectivity fraction: 1.0000
- Minimum gateway formation coverage: 1.0000
- Minimum difference from fixed index: 0.7963
- Complete receiver coverage: `1`
- Cross-formation routes observed: `1`
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
