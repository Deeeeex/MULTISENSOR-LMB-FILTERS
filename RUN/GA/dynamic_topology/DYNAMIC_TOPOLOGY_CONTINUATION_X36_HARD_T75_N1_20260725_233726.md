# Dynamic-topology oracle-gap screen

- Preset: `x36-hard`
- Seeds: `7`
- Generated: 2026-07-25 23:37:26
- Decision status: `stop-negligible-observed-gain`

- Focus window: `teacher-scale-pressure`, steps `[75 77]`

- Analysis window: steps `[75 77]`
- Conditional continuation: all arms share the same static-prefix local posterior at `t=75`

Mixture-aware results use the repository componentwise powered-GM KLA approximation. This preserves multiple modes but is not an exact arbitrary-mixture density power.

| Arm | E-OSPA | Worst node | Posterior disagreement | MAP-set disagreement | Card. error | Attempted bytes | Delivered bytes | Edges | Churn | Infeasible | Policy s | Total s |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| All-time geometry static | 105.4654 | 114.8065 | 0.6973 | 77.5102 | 11.8796 | 14264088 | 13659920 | 44.00 | 0.0000 | 0.0000 | 0.00 | 78.79 |
| Reliability dynamic | 105.3501 | 114.8067 | 0.6965 | 76.9306 | 11.8519 | 14383968 | 13975432 | 44.00 | 0.0150 | 0.0000 | 28.13 | 111.37 |
| Posterior discrepancy dynamic | 105.7921 | 114.8708 | 0.6847 | 77.1978 | 11.9537 | 14245248 | 13969040 | 44.00 | 0.1007 | 0.0000 | 98.49 | 178.43 |
| Pure current task-risk teacher | 105.0882 | 114.8065 | 0.6987 | 75.9922 | 11.7870 | 14292720 | 13944072 | 44.00 | 0.0444 | 0.0000 | 122.42 | 202.90 |
| Balanced current task-risk teacher | 105.0898 | 114.8065 | 0.6980 | 75.9538 | 11.7870 | 14299800 | 13951152 | 44.00 | 0.0444 | 0.0000 | 123.56 | 205.60 |
| Tail-risk current task-risk teacher | 105.1308 | 114.8065 | 0.6958 | 76.0281 | 11.7963 | 14302176 | 13893640 | 44.00 | 0.0444 | 0.0000 | 125.51 | 208.39 |

## Focus-window result

| Arm | Focus E-OSPA | Focus posterior disagreement | Focus MAP-set disagreement | Focus card. error |
|:--|--:|--:|--:|--:|
| All-time geometry static | 105.4654 | 0.6973 | 77.5102 | 11.8796 |
| Reliability dynamic | 105.3501 | 0.6965 | 76.9306 | 11.8519 |
| Posterior discrepancy dynamic | 105.7921 | 0.6847 | 77.1978 | 11.9537 |
| Pure current task-risk teacher | 105.0882 | 0.6987 | 75.9922 | 11.7870 |
| Balanced current task-risk teacher | 105.0898 | 0.6980 | 75.9538 | 11.7870 |
| Tail-risk current task-risk teacher | 105.1308 | 0.6958 | 76.0281 | 11.7963 |

## Registered gate readout

- Constraint-eligible arms: `6`
- Best observed arm: `Pure current task-risk teacher`
- Best observed focus E-OSPA: 105.0882
- Best observed gain vs static: 0.36%
- Minimum practical tracking gain: 5.00%
- Best observed byte mismatch vs static: 0.20%
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
