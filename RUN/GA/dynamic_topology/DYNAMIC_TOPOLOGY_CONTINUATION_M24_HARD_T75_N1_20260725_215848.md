# Dynamic-topology oracle-gap screen

- Preset: `m24-hard`
- Seeds: `7`
- Generated: 2026-07-25 21:58:48
- Decision status: `best-observed-screening`

- Focus window: `teacher-handover-and-blockage`, steps `[75 75]`

- Analysis window: steps `[75 75]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

- Metric note: this exploratory report predates boundary-aware churn
  accounting, so its churn value excludes the `t=74` to `t=75` transition.
  The selected strategy was rerun with corrected accounting in
  `DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260725_222820.md`.

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Edges | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| All-time geometry static | 25.5087 | 42.6616 | 0.8090 | 27.9094 | 0.9167 | 2808536 | 2611128 | 29.00 | 0.0000 | 0.0000 | 0.00 | 8.75 |
| Pure current task-risk teacher | 22.6428 | 42.7194 | 0.8357 | 26.2878 | 0.7500 | 2817824 | 2469840 | 29.00 | 0.0000 | 0.0000 | 21.34 | 30.17 |
| Balanced current task-risk teacher | 22.6428 | 42.7194 | 0.8357 | 26.2878 | 0.7500 | 2817824 | 2469840 | 29.00 | 0.0000 | 0.0000 | 21.35 | 30.48 |
| Tail-risk current task-risk teacher | 22.6428 | 42.7194 | 0.8357 | 26.2878 | 0.7500 | 2817824 | 2469840 | 29.00 | 0.0000 | 0.0000 | 21.33 | 30.31 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| All-time geometry static | 25.5087 | 0.8090 | 27.9094 | 0.9167 |
| Pure current task-risk teacher | 22.6428 | 0.8357 | 26.2878 | 0.7500 |
| Balanced current task-risk teacher | 22.6428 | 0.8357 | 26.2878 | 0.7500 |
| Tail-risk current task-risk teacher | 22.6428 | 0.8357 | 26.2878 | 0.7500 |

## Registered gate readout

- Best observed arm: `Pure current task-risk teacher`
- Best observed focus E-OSPA: 22.6428
- Best observed gain vs static: 11.23%
- Best observed byte mismatch vs static: 0.33%
- Oracle consensus gain: NaN%
- Oracle tracking gain: NaN%
- Analytic share of static-to-oracle gain: NaN
- Diagnostic reference dominated: `0`
- Attempted-byte mismatch: NaN%
- Recommendation: A communication-matched best observed strategy exists, but this remains a bounded screening result.

## Evidence limits

- This runner isolates topology: every active edge sends the same heavy posterior every step; event triggering and payload compression are disabled.
- A one- or three-seed run is a software/runtime screen, not a paper-level effect estimate. The registered screening gate needs at least 10 paired seeds; the held-out claim needs 30.
- Equal edge budgets do not guarantee equal bytes. The table therefore reports attempted payload bytes explicitly.
- This is a conditional continuation experiment. It compares policy decisions after a common static prefix and does not estimate full-episode performance.
- The static arm is selected by all-time geometry and link distance; it is not an exhaustive offline performance optimum.
- Exact one-step action enumeration is not a closed-loop upper bound. If it is dominated by a deployable arm, it cannot justify a learned teacher or an analytic-sufficiency claim.
