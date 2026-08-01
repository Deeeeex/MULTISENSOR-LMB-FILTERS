# Dynamic-topology oracle-gap screen

- Preset: `x36-clean-scale`
- Seeds: `53`
- Generated: 2026-08-01 00:00:55
- Evidence split: `development`
- Rejected learned artifact override: `0`
- Decision status: `insufficient-arms`
- Primary attribution family: `auto`
- Available attribution families: ``

- Focus window: `clean-scale-handover-and-blockage`, steps `[75 83]`

- Analysis window: steps `[75 83]`
- Conditional continuation: all arms share the same static-prefix local posterior and its recorded predecision selected/delivered topology history at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Undirected edges | Directed routes | Attempts | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Adaptive dominant strict-consensus reference-cap routing (regret=0.75, payload=2.0% aggregate target, alpha=0.70, epsilon=0.05) | 31.0024 | 52.8315 | 1.0471 | 29.6093 | 1.2191 | 41312120 | 39882992 | 52.56 | 58.11 | 523 | 0.3532 | 0.0000 | 1727.55 | 2409.27 |

## Analysis-window route-attribution diagnostics

| Arm | Boundary-inclusive churn | Prefix receiver changes | Within-window receiver changes | Distinct maps | Different from fixed index | Receiver coverage | Unique senders / receiver | Cross-formation routes |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| Adaptive dominant strict-consensus reference-cap routing (regret=0.75, payload=2.0% aggregate target, alpha=0.70, epsilon=0.05) | 0.3532 | 0.9722 | 0.3819 | 9.00 | 0.7037 | 1.0000 | 2.81 | 0.1033 |

When continuation is used, boundary-inclusive churn also contains the one-time transition from the common static prefix. Only the within-window receiver-change rate and distinct-map count establish that an arm actually changes routes during the evaluated window.

## Gateway and selected-routing diagnostics

| Arm | Gateway-only changes | Gateway maps | Instant weak | Instant strong | Window union weak | Window union strong | Rooted tree | Pairwise union strong | Previous graph executed | Formation coverage |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Adaptive dominant strict-consensus reference-cap routing (regret=0.75, payload=2.0% aggregate target, alpha=0.70, epsilon=0.05) | 0.0000 | 1.00 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 0.0000 | 1.0000 | 1.0000 | 1.0000 |

Gateway-only metrics mask out the synchronized intra-formation backbone. Window-union connectivity pools every focus-time graph and is descriptive only. Pairwise union strong is the fraction of focus steps whose current formation tree plus its immediately preceding graph is strongly connected; the structured tree attribution gate requires 1.0 and also requires that the preceding graph was actually executed, rather than merely supplied as the first-step bootstrap.

## Rolling-B3 selected and delivered diagnostics

| Arm | Selected sensor B3 | Selected sensor windows | Selected formation B3 | Selected formation windows | Delivered sensor B3 | Delivered sensor windows | Delivered formation B3 | Delivered formation windows | Policy sensor check | Policy formation check | One-step reserve pass | One-step checks | Joint projection | Online payload cap | Payload pass | Emergency | Repair |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Adaptive dominant strict-consensus reference-cap routing (regret=0.75, payload=2.0% aggregate target, alpha=0.70, epsilon=0.05) | 1.0000 | 9 | 1.0000 | 9 | 1.0000 | 9 | 1.0000 | 9 | 1.0000 | 1.0000 | 1.0000 | 9 | 1.0000 | 1.0000 | 1.0000 | 0.0000 | 0.0000 |

Selected B3 measures the route accepted by the scheduler. Delivered B3 is computed from messages that actually arrived after link drops. Neither quantity is labelled as an effective KLA graph: a delivered message only becomes an effective fusion input when the receiver assigns it positive fusion weight. The one-step reserve column is a current-geometry topology check, not a recursive feasibility certificate.

## Scale-risk gate diagnostics

| Arm | Minimum objective advantage | Mean objective advantage | Objective floor | Fallbacks / step | Maximum fallbacks | Activation | Payload-rejected fallbacks / step | Global payload pass |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| Adaptive dominant strict-consensus reference-cap routing (regret=0.75, payload=2.0% aggregate target, alpha=0.70, epsilon=0.05) | -0.2750 | 0.2398 | -0.6124 | 0.2222 | 1 | 0.2222 | 0.0000 | 1.0000 |

The objective advantage is computed from current posterior and link features relative to the registered fixed-index dominant map. A fallback replaces only a below-floor formation and is accepted only while the global current-state payload target remains satisfied. No truth or future outcome enters this gate.

## One-step consensus projection diagnostics

| Arm | Candidate risk | Reference risk | Selected risk | Selected gap | Fallbacks / step | Maximum fallbacks | Minimum feasible actions | Primary feasible | Payload relaxation | Constraint pass |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Adaptive dominant strict-consensus reference-cap routing (regret=0.75, payload=2.0% aggregate target, alpha=0.70, epsilon=0.05) | 1.085946 | 1.049008 | 1.046955 | -0.198% | 3.7778 | 5 | 2 | 0 | 0.3333 | 1.0000 |

These risks integrate the registered one-round heavy-fusion posterior-summary disagreement over current independent link deliveries. The projection is truth-free and exact for that one-round model; it is not reported as a recursive tracking guarantee.

## Focus-window result

| Arm | Focus E-OSPA | Focus worst node | Focus attempted bytes | Focus route changes | Focus maps | Focus coverage | Different from fixed index | Focus cross-formation | Focus infeasible | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| Adaptive dominant strict-consensus reference-cap routing (regret=0.75, payload=2.0% aggregate target, alpha=0.70, epsilon=0.05) | 31.0024 | 52.8315 | 41312120 | 0.3819 | 9.00 | 1.0000 | 0.7037 | 0.1033 | 0.0000 | 1.0471 | 29.6093 | 1.2191 |

## Per-seed tail-node diagnostic

| Seed | Arm | Worst sensor | Sensor E-OSPA | Network mean E-OSPA |
|--:|:--|--:|--:|--:|
| 53 | Adaptive dominant strict-consensus reference-cap routing (regret=0.75, payload=2.0% aggregate target, alpha=0.70, epsilon=0.05) | 8 | 52.8315 | 31.0024 |

The tail sensor is identified separately for every paired seed and arm; sensor identities must not be averaged across seeds.

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

## Privileged diagnostic-oracle readout

- Available: `0`
- Arm count: `0`
- Arms: ``
- Best diagnostic arm: ``
- Best diagnostic focus E-OSPA: NaN
- Best diagnostic attempted bytes: NaN
- Excluded from primary strategy selection: `1`

These truth-assisted arms quantify action-space headroom only. They cannot become the reported deployable strategy, select a validation model, or support a generalization claim.

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

## Structured formation-tree attribution readout

- Candidate arm: ``
- Available candidates: `0`
- Candidate source weight: NaN
- Scenario contract valid: `1`
- Formations: `6`
- Sensors per formation: `6`
- Expected cross-formation message fraction: 0.138889
- Required scheduled-tree phases: `36`
- Eligible scheduled-tree controls: `0`
- Unique registered scheduled-tree phases: `0`
- Fixed-index control available: `0`
- Link-tree control available: `0`
- Complete matched control set: `0`
- Matched reference: ``
- Per-seed reference arms: ``
- Gain vs matched control: NaN%
- Gain vs matched control by seed: `[]`
- Positive gain on every seed: `0`
- Tail-safe vs same per-seed reference: `0`
- Maximum attempted-byte mismatch: NaN%
- Communication fair: `0`
- Exact cross-formation budget every seed: `0`
- Exactly one directed message per receiver at every focus step and seed: `0`
- Exactly G-1 cross-formation sensor edges at every focus step and seed: `0`
- Rooted formation tree at every focus step and seed: `0`
- Pairwise previous/current union strongly connected at every focus step and seed: `0`
- Preceding focus-boundary graph actually executed: `0`
- Complete receiver coverage: `0`
- Distinct maps on every seed: `0`
- Within-window route changes on every seed: `0`
- Minimum difference from fixed index: NaN
- Candidate structural contract passed: `0`
- Passes structured-tree attribution gate: `0`
- Structured-tree status: `unavailable`

## Rolling-safe matched-control attribution readout

- Candidate arm: ``
- Candidate mode: ``
- Available candidates: `0`
- Observed candidate instances: `0`
- Duplicate canonical candidates: ``
- Selected candidate mode duplicated: `0`
- Candidate source weight: NaN
- Scenario contract valid: `1`
- Formations: `6`
- Expected registered controls: `0`
- Observed control instances: `0`
- Uniquely observed controls: `0`
- Contract-valid controls: `0`
- Missing controls: ``
- Duplicate controls: ``
- Invalid controls: ``
- Complete matched control set: `0`
- Scheduled behavior contract passed: `0`
- Collapsed scheduled behaviors: ``
- Candidate contract passed: `0`
- Selected-route safety passed: `0`
- Observable policy contract passed: `0`
- Joint current/successor projection used: `0`
- Online payload constraint enforced: `NaN`
- Maturity denominators matched: `0`
- Maturity denominators complete: `0`
- Maturity fields: ``
- Candidate maturity counts (rows=fields): `[]`
- Control maturity counts (rows=fields): `[]`
- Delivered-route safety did not regress: `0`
- Matched reference: ``
- Per-seed reference arms: ``
- Per-seed tracking-reference tie counts: `[]`
- Candidate/reference E-OSPA by seed: `[]` / `[]`
- Candidate/reference worst-sensor E-OSPA by seed: `[]` / `[]`
- Candidate/reference attempted bytes by seed: `[]` / `[]`
- Candidate/reference delivered sensor safety by seed: `[]` / `[]`
- Candidate/reference delivered formation safety by seed: `[]` / `[]`
- Gain vs matched control: NaN%
- Gain vs matched control by seed: `[]`
- Positive gain on every seed: `0`
- At least registered gain on every seed: `0`
- Tail-safe vs same per-seed reference: `0`
- Maximum attempted-byte mismatch: NaN%
- Attempted bytes matched within 2%: `0`
- At most 102% reference bytes every seed: `0`
- Strict tracking-byte Pareto every seed: `0`
- Passes rolling-safe attribution gate: `0`
- Rolling-safe status: `unavailable`

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
- A fresh full run uses the same registered static graph for every arm as its pre-step bootstrap topology. A cached continuation instead restores the behavior filter's actual selected and delivered directed histories, bound to the pre-decision posterior snapshot. Neither case claims that the evaluated policy selected or fused over an unexecuted bootstrap graph.
- The G-by-M scheduled-tree family is the complete registered cyclic-path root/receiver-endpoint phase sweep. It does not enumerate every rooted arborescence or every sender/receiver endpoint assignment; attribution is therefore against strong pre-registered controls, not the full non-learned action space.
- The rolling-B3 matched family is complete only relative to its registered constant-quota cyclic chunks, sparse burst phases and one rolling-constrained link-aware online scorer. It is not an exhaustive online oracle or a recursive feasibility certificate.
- The default rolling screen fixes one directed message per receiver and applies the 2% attempted-byte fairness gate to actual focus-window totals. The optional online per-step payload projection cap is disabled when `rollingPayloadProjectionToleranceFraction=inf`; the report records whether it was active for every arm.
- Sparse directed routing is evaluated as a Pareto arm: it must use no more attempted payload bytes than static (within 2% accounting tolerance) and must beat both static and local on mean tracking. It is not required to match the static edge count.
- This is a conditional continuation experiment. It compares policy decisions after a common static prefix and does not estimate full-episode performance.
- The static arm is selected by all-time geometry and link distance; it is not an exhaustive offline performance optimum.
- Exact one-step action enumeration is not a closed-loop upper bound. If it is dominated by a deployable arm, it cannot justify a learned teacher or an analytic-sufficiency claim.
