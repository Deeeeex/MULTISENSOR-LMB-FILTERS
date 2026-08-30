# V162--V165 decision: complete-label rescue needs a learned safe selector

## Decision

Retain bounded one-hop transfer of complete Bernoulli GM labels as the
estimator repair mechanism, but do not promote the V162 minimum-risk source
rule.  The next method must learn the value and risk of each observable
receiver--source--label action, expose a calibrated lower confidence bound,
and allow a safe projection to abstain or select at most `K` actions under the
remaining byte budget.

Do not continue either of the following paths:

- copying source existence evidence while retaining receiver spatial content;
- tuning another fixed scalar combination of posterior risk, receiver
  compatibility, neighbor-medoid agreement, or a smaller fixed `K`.

These are development decisions on the opened X36 seed-211 t=72 H=8 window.
They do not authorize a validation, generalization, or paper result.

## Paired recursive result that opened the issue

V162 uses the V105 protected dynamic route and requests at most four complete
one-hop labels using only current per-label posterior Bayes risk.  Against the
same-run static full-payload baseline it produced:

| Metric | Static | V162 | Relative gain |
|:--|--:|--:|--:|
| Mean E-OSPA | 84.037151 | 76.322368 | +9.180% |
| Mean matched-position RMSE | 59.967347 | 55.219073 | +7.918% |
| Window consensus E-OSPA | 61.383329 | 54.318880 | +11.509% |
| Attempted bytes | 28,578,864 | 27,876,280 | +2.458% saving |

All six formation E-OSPA gains are positive, but the strict local-RMSE gate
fails: F3 is `-29.799%` and F5 is `-2.708%`.  V105 attribution shows that these
two deficits already exist in the protected base route.  V162 repairs F1, F2,
and F6 strongly; it does not cause the F3/F5 deficits, but its original opened
trigger cells do not repair them either.

## Mechanism localization

V163 replays the F3/F5 cells before recursion.  The frozen V162 minimum-risk
Top-4 rule is useful for F3 but harmful for all 12 tested F5 receiver-time
cells.  A separate truth-scored mechanism oracle then enumerates the same
currently reachable one-hop complete labels while requiring every greedy
action to have nonnegative immediate E-OSPA gain and positive immediate RMSE
gain.

The oracle finds four joint-positive actions in every F5 cell (`48` actions in
total).  Their summed immediate E-OSPA/RMSE gains are `+4.631484/+49.816299`.
When combined with the analytic F3 repair, the projected formation RMSE gains
become:

`[25.846, 5.7725, 14.275, 1.789, 5.7265, 23.294]%`.

The projected attempted-byte saving remains positive at `+0.293%`.  Hence the
physical one-hop information and the complete-label replacement mechanism are
sufficient; the remaining failure is action selection.

## Rejected analytic repairs

### Bernoulli branch split

V164 keeps the V162 labels and sources fixed, but for shared labels either
copies only source existence evidence or admits the source spatial density
through 95%, 99%, or 99.9% chi-square compatibility gates.  All variants fail.
Existence-only updates alter MAP cardinality and the set of extracted labels,
so retaining the old spatial density does not preserve matched-position RMSE.
The projected mean RMSE gain falls below zero and the worst formation loss is
larger than `-227%`.  Existence and spatial branches may be modeled separately,
but they cannot be updated independently without accounting for the downstream
MAP extraction decision.

### Fixed observable source scores and fixed action budgets

V165 compares minimum risk, receiver compatibility, neighbor medoid,
maximum local support, credibility-weighted risk, and two chi-square-gated
source rules.  The best fixed rule, credibility-weighted risk, reduces harmful
RMSE cells from `13` to `4` and narrows the worst formation projection from
`-13.960%` to `-2.427%`, but it still fails.  Freezing the same score and
reducing the action budget to `K=1,2,3` also fails; the best projected minimum
formation RMSE gain remains about `-2.415%`.

The residual is therefore not explained by an excessive fixed action count.
It is a conditional value-estimation problem: a small subset of actions with
similar scalar risk and compatibility summaries have different joint effects
on cardinality and localization.

## Next method contract

The next selector should operate on a receiver--source--label candidate graph.
Each candidate node contains only online-observable quantities:

- receiver/source existence probability, covariance, mixture summaries, and
  association evidence;
- receiver--source position/velocity disagreement and Chernoff or Mahalanobis
  compatibility;
- agreement with other currently reachable sources carrying the same label;
- current physical-link reliability, source/receiver formation roles, label
  age, and complete-label payload bytes;
- the remaining per-step communication budget and recent protection history.

A permutation-invariant DeepSets or message-passing encoder should predict two
action values and their uncertainty: expected E-OSPA reduction and expected
matched-position RMSE reduction.  Training labels may use truth in development
simulations; truth, future measurements, and alternative-arm state are
forbidden at inference.

The policy does not directly emit a posterior or an unconstrained graph.  A
separate safe projection chooses a unique-label subset with:

- lower confidence bounds for both predicted gains greater than zero;
- at most `K` complete labels and attempted bytes within current headroom;
- no reduction of the protected effective KLA carrier connectivity;
- abstention as the mandatory fallback when no candidate clears the bounds.

The first data split must be by scenario/seed, not by individual candidate
rows.  Development scenes may include the opened X36 window and additional
non-radial presets; evaluation must reserve unseen seeds and at least one
geometry family.  Only after a frozen model passes the recursive X36 gate may
it be run unchanged on M24 and the held-out non-radial/scale suite.

## Evidence files

- `evidence/tracking_aligned_v162/observable_one_hop_risk_label/x36_t72_h8/OBSERVABLE_ONE_HOP_RISK_LABEL_V162_X36_T72_RESULT.md`
- `evidence/tracking_aligned_v162/v105_rmse_attribution/x36_t72_h8/PROTECTION_ONLY_H8_V105_X36_T72_RESULT.md`
- `evidence/tracking_aligned_v162/observable_one_hop_risk_label/rmse_repair_preflight/OBSERVABLE_ONE_HOP_RMSE_REPAIR_V163_PREFLIGHT.md`
- `evidence/tracking_aligned_v162/observable_one_hop_risk_label/branch_selective_preflight/BRANCH_SELECTIVE_ONE_HOP_REPAIR_V164_PREFLIGHT.md`
- `evidence/tracking_aligned_v162/observable_one_hop_risk_label/consensus_source_preflight/CONSENSUS_AWARE_ONE_HOP_SOURCE_V165_PREFLIGHT.md`
