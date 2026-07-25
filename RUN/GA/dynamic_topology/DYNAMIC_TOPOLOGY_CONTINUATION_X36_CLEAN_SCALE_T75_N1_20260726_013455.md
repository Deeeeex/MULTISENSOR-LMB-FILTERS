# Dynamic-topology oracle-gap screen

- Preset: `x36-clean-scale`
- Seeds: `7`
- Generated: 2026-07-26 01:34:55
- Decision status: `stop-negligible-observed-gain`

- Focus window: `clean-scale-handover-and-blockage`, steps `[75 77]`

- Analysis window: steps `[75 77]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Edges | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| All-time geometry static | 46.3716 | 63.8858 | 1.0060 | 35.7744 | 2.3981 | 19957824 | 19131128 | 44.00 | 0.0000 | 0.0000 | 0.00 | 107.10 |
| Reliability dynamic | 46.2599 | 63.8827 | 1.0072 | 34.9606 | 2.3796 | 19994952 | 19384424 | 44.00 | 0.0150 | 0.0000 | 32.33 | 142.15 |
| Posterior discrepancy dynamic | 48.4618 | 65.4138 | 0.9695 | 36.7017 | 2.6111 | 19910736 | 19154176 | 44.00 | 0.0730 | 0.0000 | 114.70 | 225.01 |
| Pure current task-risk teacher | 45.9258 | 68.4314 | 1.0061 | 35.8124 | 2.3704 | 20016096 | 19255144 | 44.00 | 0.0299 | 0.0000 | 128.10 | 237.54 |
| Balanced current task-risk teacher | 46.2599 | 63.8827 | 1.0072 | 34.9606 | 2.3796 | 19994952 | 19384424 | 44.00 | 0.0150 | 0.0000 | 134.66 | 245.61 |
| Tail-risk current task-risk teacher | 46.2599 | 63.8827 | 1.0072 | 34.9606 | 2.3796 | 19994952 | 19384424 | 44.00 | 0.0150 | 0.0000 | 134.30 | 244.56 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| All-time geometry static | 46.3716 | 1.0060 | 35.7744 | 2.3981 |
| Reliability dynamic | 46.2599 | 1.0072 | 34.9606 | 2.3796 |
| Posterior discrepancy dynamic | 48.4618 | 0.9695 | 36.7017 | 2.6111 |
| Pure current task-risk teacher | 45.9258 | 1.0061 | 35.8124 | 2.3704 |
| Balanced current task-risk teacher | 46.2599 | 1.0072 | 34.9606 | 2.3796 |
| Tail-risk current task-risk teacher | 46.2599 | 1.0072 | 34.9606 | 2.3796 |

## Registered gate readout

- Constraint-eligible arms: `6`
- Best observed arm: `Pure current task-risk teacher`
- Best observed focus E-OSPA: 45.9258
- Best observed gain vs static: 0.96%
- Minimum practical tracking gain: 5.00%
- Best observed byte mismatch vs static: 0.29%
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
