# Dynamic-topology oracle-gap screen

- Preset: `m24-hard`
- Seeds: `7`
- Generated: 2026-07-26 10:41:51
- Decision status: `dynamic-attribution-gate-failed`

- Focus window: `teacher-handover-and-blockage`, steps `[75 77]`

- Analysis window: steps `[75 77]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Undirected edges | Directed routes | Attempts | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed round robin control (phase=1, w=0.50) | 22.3297 | 34.8109 | 0.8982 | 24.2645 | 0.7222 | 3471264 | 3420400 | 24.00 | 24.00 | 72 | 0.8442 | 0.0000 | 0.00 | 25.90 |
| Directed fixed gateway control (phase=1, w=0.50) | 23.4224 | 41.7655 | 0.8664 | 26.9219 | 0.8333 | 3236208 | 3030152 | 24.00 | 24.00 | 72 | 0.8133 | 0.0000 | 0.00 | 24.92 |
| Directed fixed gateway control (phase=2, w=0.50) | 25.3853 | 48.5163 | 0.8468 | 28.7625 | 0.9167 | 3289584 | 3108088 | 24.00 | 24.00 | 72 | 0.8133 | 0.0000 | 0.00 | 24.94 |
| Directed fixed gateway control (phase=3, w=0.50) | 24.0859 | 42.6697 | 0.8606 | 26.3745 | 0.8333 | 3291480 | 3058376 | 24.00 | 24.00 | 72 | 0.8133 | 0.0000 | 0.00 | 24.86 |
| Directed fixed gateway control (phase=4, w=0.50) | 27.6078 | 44.5771 | 0.8233 | 29.0536 | 1.0833 | 3459168 | 3459168 | 24.00 | 24.00 | 72 | 0.8133 | 0.0000 | 0.00 | 25.56 |
| Directed fixed gateway control (phase=5, w=0.50) | 24.9028 | 44.5427 | 0.8815 | 26.1862 | 0.8750 | 3176376 | 2977208 | 24.00 | 24.00 | 72 | 0.8133 | 0.0000 | 0.00 | 25.65 |
| Directed fixed gateway control (phase=6, w=0.50) | 24.3433 | 45.8803 | 0.8621 | 28.0510 | 0.8889 | 3134328 | 2929424 | 24.00 | 24.00 | 72 | 0.8133 | 0.0000 | 0.00 | 27.19 |
| Directed rotating gateway control (phase=1, w=0.50) | 25.4285 | 44.5621 | 0.8470 | 29.1362 | 0.9444 | 3348048 | 3195456 | 24.00 | 24.00 | 72 | 0.8734 | 0.0000 | 0.00 | 26.68 |
| Directed rotating gateway control (phase=2, w=0.50) | 24.1873 | 40.4864 | 0.8379 | 29.1295 | 0.9167 | 3456936 | 3409456 | 24.00 | 24.00 | 72 | 0.8734 | 0.0000 | 0.00 | 25.81 |
| Directed rotating gateway control (phase=3, w=0.50) | 23.5871 | 42.7172 | 0.8710 | 27.8259 | 0.8333 | 3251136 | 2915800 | 24.00 | 24.00 | 72 | 0.8734 | 0.0000 | 0.00 | 24.52 |
| Directed rotating gateway control (phase=4, w=0.50) | 26.2993 | 44.5339 | 0.8411 | 29.6326 | 1.0278 | 3338760 | 3037440 | 24.00 | 24.00 | 72 | 0.8734 | 0.0000 | 0.00 | 24.60 |
| Directed rotating gateway control (phase=5, w=0.50) | 25.1708 | 44.4153 | 0.8690 | 28.7359 | 0.9444 | 3329064 | 3081296 | 24.00 | 24.00 | 72 | 0.8734 | 0.0000 | 0.00 | 24.80 |
| Directed rotating gateway control (phase=6, w=0.50) | 24.4428 | 40.5673 | 0.8483 | 29.0873 | 0.9028 | 3383016 | 3280616 | 24.00 | 24.00 | 72 | 0.8734 | 0.0000 | 0.00 | 24.88 |
| Directed link-aware gateway control (w=0.50) | 22.3611 | 37.9768 | 0.8602 | 24.8995 | 0.7361 | 3337872 | 3140320 | 23.00 | 24.00 | 72 | 0.7891 | 0.0000 | 0.00 | 24.99 |
| Novelty-gated formation gateway (v1, w=0.50) | 21.9550 | 35.1839 | 0.8816 | 25.9232 | 0.7222 | 3409320 | 3308264 | 23.67 | 24.00 | 72 | 0.8662 | 0.0000 | 16.98 | 42.53 |

## Analysis-window route-attribution diagnostics

| Arm | Boundary-inclusive churn | Prefix receiver changes | Within-window receiver changes | Distinct maps | Different from fixed index | Receiver coverage | Unique senders / receiver | Cross-formation routes |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed round robin control (phase=1, w=0.50) | 0.8442 | 1.0000 | 1.0000 | 3.00 | 0.7778 | 1.0000 | 3.00 | 0.0000 |
| Directed fixed gateway control (phase=1, w=0.50) | 0.8133 | 1.0000 | 0.8333 | 3.00 | 0.8333 | 1.0000 | 2.67 | 0.1667 |
| Directed fixed gateway control (phase=2, w=0.50) | 0.8133 | 1.0000 | 0.8333 | 3.00 | 0.8333 | 1.0000 | 2.67 | 0.1667 |
| Directed fixed gateway control (phase=3, w=0.50) | 0.8133 | 1.0000 | 0.8333 | 3.00 | 0.7778 | 1.0000 | 2.67 | 0.1667 |
| Directed fixed gateway control (phase=4, w=0.50) | 0.8133 | 1.0000 | 0.8333 | 3.00 | 0.7778 | 1.0000 | 2.67 | 0.1667 |
| Directed fixed gateway control (phase=5, w=0.50) | 0.8133 | 1.0000 | 0.8333 | 3.00 | 0.8333 | 1.0000 | 2.67 | 0.1667 |
| Directed fixed gateway control (phase=6, w=0.50) | 0.8133 | 1.0000 | 0.8333 | 3.00 | 0.8333 | 1.0000 | 2.67 | 0.1667 |
| Directed rotating gateway control (phase=1, w=0.50) | 0.8734 | 1.0000 | 1.0000 | 3.00 | 0.8333 | 1.0000 | 3.00 | 0.1667 |
| Directed rotating gateway control (phase=2, w=0.50) | 0.8734 | 1.0000 | 1.0000 | 3.00 | 0.7778 | 1.0000 | 3.00 | 0.1667 |
| Directed rotating gateway control (phase=3, w=0.50) | 0.8734 | 1.0000 | 1.0000 | 3.00 | 0.8333 | 1.0000 | 3.00 | 0.1667 |
| Directed rotating gateway control (phase=4, w=0.50) | 0.8734 | 1.0000 | 1.0000 | 3.00 | 0.8333 | 1.0000 | 3.00 | 0.1667 |
| Directed rotating gateway control (phase=5, w=0.50) | 0.8734 | 1.0000 | 1.0000 | 3.00 | 0.7778 | 1.0000 | 3.00 | 0.1667 |
| Directed rotating gateway control (phase=6, w=0.50) | 0.8734 | 1.0000 | 1.0000 | 3.00 | 0.8333 | 1.0000 | 3.00 | 0.1667 |
| Directed link-aware gateway control (w=0.50) | 0.7891 | 1.0000 | 0.8333 | 3.00 | 0.8333 | 1.0000 | 2.67 | 0.1667 |
| Novelty-gated formation gateway (v1, w=0.50) | 0.8662 | 1.0000 | 1.0000 | 3.00 | 0.7917 | 1.0000 | 3.00 | 0.0972 |

When continuation is used, boundary-inclusive churn also contains the one-time transition from the common static prefix. Only the within-window receiver-change rate and distinct-map count establish that an arm actually changes routes during the evaluated window.

## Gateway and effective-graph diagnostics

| Arm | Gateway-only changes | Gateway maps | Instant weak | Instant strong | Union weak | Union strong | Formation coverage |
|:--|--:|--:|--:|--:|--:|--:|--:|
| Directed round robin control (phase=1, w=0.50) | 0.0000 | 1.00 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| Directed fixed gateway control (phase=1, w=0.50) | 0.0000 | 1.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Directed fixed gateway control (phase=2, w=0.50) | 0.0000 | 1.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Directed fixed gateway control (phase=3, w=0.50) | 0.0000 | 1.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Directed fixed gateway control (phase=4, w=0.50) | 0.0000 | 1.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Directed fixed gateway control (phase=5, w=0.50) | 0.0000 | 1.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Directed fixed gateway control (phase=6, w=0.50) | 0.0000 | 1.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Directed rotating gateway control (phase=1, w=0.50) | 0.3333 | 3.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Directed rotating gateway control (phase=2, w=0.50) | 0.3333 | 3.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Directed rotating gateway control (phase=3, w=0.50) | 0.3333 | 3.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Directed rotating gateway control (phase=4, w=0.50) | 0.3333 | 3.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Directed rotating gateway control (phase=5, w=0.50) | 0.3333 | 3.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Directed rotating gateway control (phase=6, w=0.50) | 0.3333 | 3.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Directed link-aware gateway control (w=0.50) | 0.0000 | 1.00 | 1.0000 | 0.0000 | 1.0000 | 0.0000 | 1.0000 |
| Novelty-gated formation gateway (v1, w=0.50) | 0.1458 | 3.00 | 0.0000 | 0.0000 | 1.0000 | 0.0000 | 0.7500 |

Gateway-only metrics mask out the synchronized intra-formation round-robin backbone. Union connectivity is computed on the directed formation graph over the complete focus window; the attribution gate requires at least weak connectivity.

## Focus-window result

| Arm | Focus E-OSPA | Focus worst node | Focus attempted bytes | Focus route changes | Focus maps | Focus coverage | Different from fixed index | Focus cross-formation | Focus infeasible | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Directed round robin control (phase=1, w=0.50) | 22.3297 | 34.8109 | 3471264 | 1.0000 | 3.00 | 1.0000 | 0.7778 | 0.0000 | 0.0000 | 0.8982 | 24.2645 | 0.7222 |
| Directed fixed gateway control (phase=1, w=0.50) | 23.4224 | 41.7655 | 3236208 | 0.8333 | 3.00 | 1.0000 | 0.8333 | 0.1667 | 0.0000 | 0.8664 | 26.9219 | 0.8333 |
| Directed fixed gateway control (phase=2, w=0.50) | 25.3853 | 48.5163 | 3289584 | 0.8333 | 3.00 | 1.0000 | 0.8333 | 0.1667 | 0.0000 | 0.8468 | 28.7625 | 0.9167 |
| Directed fixed gateway control (phase=3, w=0.50) | 24.0859 | 42.6697 | 3291480 | 0.8333 | 3.00 | 1.0000 | 0.7778 | 0.1667 | 0.0000 | 0.8606 | 26.3745 | 0.8333 |
| Directed fixed gateway control (phase=4, w=0.50) | 27.6078 | 44.5771 | 3459168 | 0.8333 | 3.00 | 1.0000 | 0.7778 | 0.1667 | 0.0000 | 0.8233 | 29.0536 | 1.0833 |
| Directed fixed gateway control (phase=5, w=0.50) | 24.9028 | 44.5427 | 3176376 | 0.8333 | 3.00 | 1.0000 | 0.8333 | 0.1667 | 0.0000 | 0.8815 | 26.1862 | 0.8750 |
| Directed fixed gateway control (phase=6, w=0.50) | 24.3433 | 45.8803 | 3134328 | 0.8333 | 3.00 | 1.0000 | 0.8333 | 0.1667 | 0.0000 | 0.8621 | 28.0510 | 0.8889 |
| Directed rotating gateway control (phase=1, w=0.50) | 25.4285 | 44.5621 | 3348048 | 1.0000 | 3.00 | 1.0000 | 0.8333 | 0.1667 | 0.0000 | 0.8470 | 29.1362 | 0.9444 |
| Directed rotating gateway control (phase=2, w=0.50) | 24.1873 | 40.4864 | 3456936 | 1.0000 | 3.00 | 1.0000 | 0.7778 | 0.1667 | 0.0000 | 0.8379 | 29.1295 | 0.9167 |
| Directed rotating gateway control (phase=3, w=0.50) | 23.5871 | 42.7172 | 3251136 | 1.0000 | 3.00 | 1.0000 | 0.8333 | 0.1667 | 0.0000 | 0.8710 | 27.8259 | 0.8333 |
| Directed rotating gateway control (phase=4, w=0.50) | 26.2993 | 44.5339 | 3338760 | 1.0000 | 3.00 | 1.0000 | 0.8333 | 0.1667 | 0.0000 | 0.8411 | 29.6326 | 1.0278 |
| Directed rotating gateway control (phase=5, w=0.50) | 25.1708 | 44.4153 | 3329064 | 1.0000 | 3.00 | 1.0000 | 0.7778 | 0.1667 | 0.0000 | 0.8690 | 28.7359 | 0.9444 |
| Directed rotating gateway control (phase=6, w=0.50) | 24.4428 | 40.5673 | 3383016 | 1.0000 | 3.00 | 1.0000 | 0.8333 | 0.1667 | 0.0000 | 0.8483 | 29.0873 | 0.9028 |
| Directed link-aware gateway control (w=0.50) | 22.3611 | 37.9768 | 3337872 | 0.8333 | 3.00 | 1.0000 | 0.8333 | 0.1667 | 0.0000 | 0.8602 | 24.8995 | 0.7361 |
| Novelty-gated formation gateway (v1, w=0.50) | 21.9550 | 35.1839 | 3409320 | 1.0000 | 3.00 | 1.0000 | 0.7917 | 0.0972 | 0.0000 | 0.8816 | 25.9232 | 0.7222 |

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
- Recommendation: The candidate does not jointly pass the gain, per-seed, tail, byte, feasibility and within-window route-change gates, including gateway-only dynamics and formation-union connectivity, against the strongest registered controls.

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

- Candidate arm: `Novelty-gated formation gateway (v1, w=0.50)`
- Available candidates: `1`
- Candidates with complete matched controls: `1`
- Structurally eligible candidates: `1`
- Strong-control reference: `best per-seed registered fixed/scheduled control across weights`
- Weight-matched reference: `best per-seed weight-matched round-robin/fixed/rotating/link-aware gateway control`
- Complete registered control set: `1`
- Candidate source weight: 0.50
- Weight-matched control set: `1`
- Matched fixed-gateway phases: `6`
- Matched rotating-gateway phases: `6`
- Required gateway phases per family: `6`
- Candidate uses cross-formation routes: `1`
- Cross-formation scheduled control set: `1`
- Action support matched: `1`
- Gain vs weight-matched control: 1.6781%
- Gain vs weight-matched control by seed: `1.6781`
- Positive matched-control gain on every seed: `1`
- Gain vs strongest control: 1.6781%
- Gain by seed: `1.6781`
- Positive gain on every seed: `1`
- Strict tail-safe vs strongest control: `0`
- Maximum attempted-byte mismatch: 1.7845%
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
- Minimum instantaneous weak-connectivity fraction: 0.0000
- Minimum gateway formation coverage: 0.7500
- Minimum difference from fixed index: 0.7917
- Complete receiver coverage: `1`
- Cross-formation routes observed: `1`
- Passes dynamic-attribution gate: `0`
- Dynamic-attribution status: `dynamic-attribution-gate-failed`

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
- Sparse directed routing is evaluated as a Pareto arm: it must use no more attempted payload bytes than static (within 2% accounting tolerance) and must beat both static and local on mean tracking. It is not required to match the static edge count.
- This is a conditional continuation experiment. It compares policy decisions after a common static prefix and does not estimate full-episode performance.
- The static arm is selected by all-time geometry and link distance; it is not an exhaustive offline performance optimum.
- Exact one-step action enumeration is not a closed-loop upper bound. If it is dominated by a deployable arm, it cannot justify a learned teacher or an analytic-sufficiency claim.
