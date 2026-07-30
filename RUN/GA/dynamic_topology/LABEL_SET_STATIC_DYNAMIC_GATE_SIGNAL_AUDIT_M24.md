# Label-set static/dynamic gate signal audit

## Scope

This training-only audit asks whether a simple predecision rule at \(t=75\)
can decide when the topology-only policy should replace the frozen CCW
control.

- Seeds: `11, 17, 19, 23, 27, 29`
- Input time: `75`
- Inputs: observable LMB posterior, geometry, current link reliability,
  registered topology history, fixed-weight GNN scores, and posterior
  complexity
- Excluded inputs: seed identifier, target truth, future measurements,
  future outcomes, and post-run statistics
- Diagnostic target: registered full-window closed-loop mean E-OSPA gain
  of topology-only versus frozen CCW
- Development/held-out/X36 opened: `0 / 0 / 0`

The target uses training truth only to test signal learnability. It is not
available to a deployed gate.

The reconstructed diagnostic history matches the continuation filter's
first-step initialization: innovation is zero, association confidence is
one, and unavailable NIS values remain `NaN`. The policy feature builder,
rather than this audit, applies its registered missing-value handling.

## Target

| Seed | Gain versus frozen CCW | Positive |
|--:|--:|--:|
| 11 | 10.880% | 1 |
| 17 | -6.267% | 0 |
| 19 | 19.404% | 1 |
| 23 | 13.839% | 1 |
| 27 | -12.026% | 0 |
| 29 | 8.849% | 1 |

Seed 23 is positive against frozen CCW but negative against the
retrospective CW control. A binary CCW/dynamic gate therefore cannot recover
the full three-arm oracle; CW, CCW, and learned topology must remain separate
actions.

## Strongest scalar associations

| Predecision signal | Spearman rho |
|:--|--:|
| Mean learned GNN edge score | -1.000 |
| Minimum learned GNN edge score | -0.943 |
| Standard deviation of learned-minus-CCW GNN score | 0.943 |
| Maximum learned-minus-CCW GNN score | 0.829 |
| Minimum learned-minus-CCW Bernoulli compatibility | -0.714 |
| Minimum learned-minus-CCW spatial overlap | -0.714 |
| Maximum across sensors of mean Bernoulli existence entropy | -0.714 |
| Minimum GM component count across sensors | -0.696 |
| Minimum learned-minus-CCW KLA compatibility score | -0.600 |
| Standard deviation of GM component count | 0.600 |
| Standard deviation of mean existence entropy | -0.600 |

The mean GNN score is perfectly rank-correlated with the six observed
training outcomes. This is a useful hypothesis, but not generalization
evidence: it was found after searching all available summaries on only six
independent seed clusters. The nested test below measures the combined
feature-selection and threshold-selection instability.

## Nested threshold test

For each outer fold, every scalar feature, threshold, and direction is fitted
using the other five seeds. The selected one-dimensional rule is then applied
once to the held-out seed.

| Held seed | Target | Prediction | Best training accuracy |
|--:|--:|--:|--:|
| 11 | 1 | 0 | 1.0 |
| 17 | 0 | 1 | 1.0 |
| 19 | 1 | 1 | 1.0 |
| 23 | 1 | 0 | 1.0 |
| 27 | 0 | 1 | 1.0 |
| 29 | 1 | 0 | 1.0 |

- Nested LOSO accuracy: `1/6 = 0.167`
- Optimistic all-seed threshold accuracy: `6/6 = 1.000`
- Optimistic selected feature: mean learned GNN edge score
- Optimistic threshold: select learned topology when mean GNN score is at
  most `0.0132304`

The gap between optimistic fitting and nested LOSO is direct evidence of
small-sample feature-selection and threshold instability. The perfect
all-seed split is therefore not used as a learned policy, and a hand-tuned
single-frame predecision threshold is rejected.

## Decision

1. Reject the single-frame scalar threshold gate.
2. Do not use the current absolute GNN graph score as confidence; its
   `taskAdvantage` diagnostic is an absolute projected-graph score, not a
   calibrated advantage versus a static action.
3. Generate common-random-number \(H=3\) graph-level returns for the explicit
   action set `{CW, CCW, learned topology}`.
4. Train a small relative value head with abstention to a frozen static
   action. Include worst-node, consensus, communication, topology churn, and
   posterior-complexity constraints.
5. Evaluate using whole-seed `fit-4 / calibrate-1 / test-1`. The low-level
   label-set scorer must also be retrained inside each outer split; otherwise
   the test seed leaks through its current-truth initialization target.
6. Re-run the selected gated policy closed-loop on each outer test seed.
   Existing arm trajectories may define diagnostic targets but may not be
   spliced into a claimed policy result.
7. Stop the gate route if this strict training audit fails; keep
   development M24, held-out M24, and X36 sealed.

## Reproducibility

- Audit implementation:
  `RUN/GA/auditLabelSetStaticDynamicGateSignalsM24.m`
- Nested scalar evaluation:
  `common/evaluateNestedOneDimensionalGateLoso.m`
- Audit MAT:
  `RUN/GA/dynamic_topology/label_set_static_dynamic_gate_signal_audit_m24.mat`
- Audit MAT SHA-256:
  `711e9a876bacb5b32dbb3d2cde127ff28164dc936030095943c2d11a977cf5bf`
