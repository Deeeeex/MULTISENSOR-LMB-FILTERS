# M24 CVaR relational graph policy v1: rejection note

## Scope

This is a development-only architecture smoke test on the six registered
training seeds `[11, 17, 19, 23, 27, 29]`. Seeds 31 and 37 remained sealed.
The test used whole-seed `fit-4 / calibrate-1 / test-1` folds, so no state
from a tested seed entered model fitting or safety calibration.

## Result

| Model | Mean top-3 node recall | Mean selected true task advantage | Nonnegative states |
|:--|--:|--:|--:|
| Ridge sentinel | 0.204 | -0.051 | 14 / 54 |
| Relational MLP | 0.228 | -0.092 | 18 / 54 |

For the seed-11 fold (`fit=[19,23,27,29]`, `calibrate=17`), the ridge and
MLP edge-task Spearman correlations were 0.581 and 0.637, respectively.
Despite this moderate row-wise ranking signal, their top-3 node recalls
were only 0.185 and 0.111. Their selected-cycle median normalized regrets
were 0.268 and 0.336. The calibrated observed-tail true-safe recalls were
below 0.5%, and the selected-tail-safe state fractions were 0.889 and
0.556.

## Diagnosis

The v1 node representation pooled posterior uncertainty, pairwise
compatibility, link and topology-history features, but omitted the local
measurement-update innovation statistics already computed by the filter.
Posterior agreement alone cannot reveal a common-mode tracking error:
several nodes can be mutually consistent and jointly wrong. The learned
argmax also amplified modest edge-regression errors into negative
cycle-level task advantage.

The v1 model is therefore rejected before any development evaluation or
H=3 learned-policy return run. Increasing MLP capacity on the same feature
contract is not authorized.

## v2 direction

The next feature contract adds a causal three-step history of local
innovation novelty, association confidence, normalized innovation squared
and NIS consistency deviation. These statistics are available immediately
after each local measurement update and before topology selection. The
exact message-count-preserving strong-cycle projector remains unchanged.
The v2 preflight must demonstrate feature replay and materially stronger
whole-seed tail-risk prediction before training shards or development
labels can be opened.
