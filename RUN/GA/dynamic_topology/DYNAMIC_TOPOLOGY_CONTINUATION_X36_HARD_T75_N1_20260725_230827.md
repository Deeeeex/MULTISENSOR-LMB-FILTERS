# Dynamic-topology oracle-gap screen

- Preset: `x36-hard`
- Seeds: `7`
- Generated: 2026-07-25 23:08:27
- Decision status: `stop-negligible-observed-gain`

- Focus window: `teacher-scale-pressure`, steps `[75 75]`

- Analysis window: steps `[75 75]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Edges | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| All-time geometry static | 107.3636 | 118.6369 | 0.6674 | 81.9788 | 12.3056 | 4689704 | 4468512 | 44.00 | 0.0000 | 0.0000 | 0.00 | 22.41 |
| Reliability dynamic | 107.3807 | 114.9030 | 0.6685 | 81.5160 | 12.3056 | 4718864 | 4543568 | 44.00 | 0.0444 | 0.0000 | 9.17 | 31.10 |
| Posterior discrepancy dynamic | 107.8652 | 114.9030 | 0.6658 | 81.7043 | 12.4167 | 4700048 | 4524752 | 44.00 | 0.1277 | 0.0000 | 29.55 | 51.42 |
| Pure current task-risk teacher | 107.2657 | 114.9030 | 0.6696 | 80.9183 | 12.2778 | 4703912 | 4528616 | 44.00 | 0.0444 | 0.0000 | 41.22 | 62.91 |
| Balanced current task-risk teacher | 107.2657 | 114.9030 | 0.6696 | 80.9183 | 12.2778 | 4703912 | 4528616 | 44.00 | 0.0444 | 0.0000 | 40.82 | 62.56 |
| Tail-risk current task-risk teacher | 107.2657 | 114.9030 | 0.6696 | 80.9183 | 12.2778 | 4703912 | 4528616 | 44.00 | 0.0444 | 0.0000 | 40.96 | 62.68 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| All-time geometry static | 107.3636 | 0.6674 | 81.9788 | 12.3056 |
| Reliability dynamic | 107.3807 | 0.6685 | 81.5160 | 12.3056 |
| Posterior discrepancy dynamic | 107.8652 | 0.6658 | 81.7043 | 12.4167 |
| Pure current task-risk teacher | 107.2657 | 0.6696 | 80.9183 | 12.2778 |
| Balanced current task-risk teacher | 107.2657 | 0.6696 | 80.9183 | 12.2778 |
| Tail-risk current task-risk teacher | 107.2657 | 0.6696 | 80.9183 | 12.2778 |

## Registered gate readout

- Constraint-eligible arms: `6`
- Best observed arm: `Pure current task-risk teacher`
- Best observed focus E-OSPA: 107.2657
- Best observed gain vs static: 0.09%
- Minimum practical tracking gain: 5.00%
- Best observed byte mismatch vs static: 0.30%
- Oracle consensus gain: NaN%
- Oracle tracking gain: NaN%
- Analytic share of static-to-oracle gain: NaN
- Diagnostic reference dominated: `0`
- Attempted-byte mismatch: NaN%
- Recommendation: The best observed communication-matched gain is below the registered 5.00% practical-effect threshold.

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
- This is a conditional continuation experiment. It compares policy decisions after a common static prefix and does not estimate full-episode performance.
- The static arm is selected by all-time geometry and link distance; it is not an exhaustive offline performance optimum.
- Exact one-step action enumeration is not a closed-loop upper bound. If it is dominated by a deployable arm, it cannot justify a learned teacher or an analytic-sufficiency claim.
