# Dynamic-topology oracle-gap screen

- Preset: `m24-hard`
- Seeds: `7`
- Generated: 2026-07-26 12:01:47
- Decision status: `insufficient-arms`

- Focus window: `teacher-handover-and-blockage`, steps `[75 77]`

- Analysis window: steps `[75 77]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Undirected edges | Directed routes | Attempts | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Strong formation cycle v3 (bridge=0.10, w=0.70) | 18.5164 | 34.6420 | 0.9168 | 20.8162 | 0.5278 | 3505704 | 3404648 | 20.00 | 24.00 | 72 | 0.5645 | 0.0000 | 17.14 | 43.12 |
| Strong formation cycle v3 (bridge=0.15, w=0.70) | 18.6705 | 34.6420 | 0.9094 | 21.1979 | 0.5417 | 3505704 | 3404648 | 20.00 | 24.00 | 72 | 0.5645 | 0.0000 | 17.28 | 43.33 |
| Strong formation cycle v3 (bridge=0.20, w=0.70) | 19.1942 | 35.9041 | 0.8999 | 21.4717 | 0.5694 | 3505704 | 3404648 | 20.00 | 24.00 | 72 | 0.5645 | 0.0000 | 17.02 | 42.90 |
| Strong formation cycle v3 (bridge=0.30, w=0.70) | 19.4955 | 43.6159 | 0.8886 | 22.1958 | 0.5972 | 3508056 | 3407000 | 20.00 | 24.00 | 72 | 0.5645 | 0.0000 | 17.01 | 43.16 |

## Analysis-window route-attribution diagnostics

| Arm | Boundary-inclusive churn | Prefix receiver changes | Within-window receiver changes | Distinct maps | Different from fixed index | Receiver coverage | Unique senders / receiver | Cross-formation routes |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| Strong formation cycle v3 (bridge=0.10, w=0.70) | 0.5645 | 1.0000 | 0.1458 | 3.00 | 0.1667 | 1.0000 | 1.29 | 0.1667 |
| Strong formation cycle v3 (bridge=0.15, w=0.70) | 0.5645 | 1.0000 | 0.1458 | 3.00 | 0.1667 | 1.0000 | 1.29 | 0.1667 |
| Strong formation cycle v3 (bridge=0.20, w=0.70) | 0.5645 | 1.0000 | 0.1458 | 3.00 | 0.1667 | 1.0000 | 1.29 | 0.1667 |
| Strong formation cycle v3 (bridge=0.30, w=0.70) | 0.5645 | 1.0000 | 0.1458 | 3.00 | 0.1667 | 1.0000 | 1.29 | 0.1667 |

When continuation is used, boundary-inclusive churn also contains the one-time transition from the common static prefix. Only the within-window receiver-change rate and distinct-map count establish that an arm actually changes routes during the evaluated window.

## Gateway and effective-graph diagnostics

| Arm | Gateway-only changes | Gateway maps | Instant weak | Instant strong | Union weak | Union strong | Formation coverage |
|:--|--:|--:|--:|--:|--:|--:|--:|
| Strong formation cycle v3 (bridge=0.10, w=0.70) | 0.1458 | 3.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Strong formation cycle v3 (bridge=0.15, w=0.70) | 0.1458 | 3.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Strong formation cycle v3 (bridge=0.20, w=0.70) | 0.1458 | 3.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| Strong formation cycle v3 (bridge=0.30, w=0.70) | 0.1458 | 3.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |

Gateway-only metrics mask out the synchronized intra-formation round-robin backbone. Union connectivity is computed on the directed formation graph over the complete focus window; the attribution gate requires at least weak connectivity.

## Focus-window result

| Arm | Focus E-OSPA | Focus worst node | Focus attempted bytes | Focus route changes | Focus maps | Focus coverage | Different from fixed index | Focus cross-formation | Focus infeasible | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Strong formation cycle v3 (bridge=0.10, w=0.70) | 18.5164 | 34.6420 | 3505704 | 0.1458 | 3.00 | 1.0000 | 0.1667 | 0.1667 | 0.0000 | 0.9168 | 20.8162 | 0.5278 |
| Strong formation cycle v3 (bridge=0.15, w=0.70) | 18.6705 | 34.6420 | 3505704 | 0.1458 | 3.00 | 1.0000 | 0.1667 | 0.1667 | 0.0000 | 0.9094 | 21.1979 | 0.5417 |
| Strong formation cycle v3 (bridge=0.20, w=0.70) | 19.1942 | 35.9041 | 3505704 | 0.1458 | 3.00 | 1.0000 | 0.1667 | 0.1667 | 0.0000 | 0.8999 | 21.4717 | 0.5694 |
| Strong formation cycle v3 (bridge=0.30, w=0.70) | 19.4955 | 43.6159 | 3508056 | 0.1458 | 3.00 | 1.0000 | 0.1667 | 0.1667 | 0.0000 | 0.8886 | 22.1958 | 0.5972 |

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

- Candidate arm: ``
- Available candidates: `0`
- Candidates with complete matched controls: `0`
- Structurally eligible candidates: `0`
- Strong-control reference: ``
- Weight-matched reference: ``
- Complete registered control set: `0`
- Candidate source weight: NaN
- Weight-matched control set: `0`
- Matched fixed-gateway phases: `0`
- Matched rotating-gateway phases: `0`
- Required gateway phases per family: `NaN`
- Candidate uses cross-formation routes: `0`
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
- Dynamic-attribution status: `unavailable`

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
- Sparse directed routing is evaluated as a Pareto arm: it must use no more attempted payload bytes than static (within 2% accounting tolerance) and must beat both static and local on mean tracking. It is not required to match the static edge count.
- This is a conditional continuation experiment. It compares policy decisions after a common static prefix and does not estimate full-episode performance.
- The static arm is selected by all-time geometry and link distance; it is not an exhaustive offline performance optimum.
- Exact one-step action enumeration is not a closed-loop upper bound. If it is dominated by a deployable arm, it cannot justify a learned teacher or an analytic-sufficiency claim.
