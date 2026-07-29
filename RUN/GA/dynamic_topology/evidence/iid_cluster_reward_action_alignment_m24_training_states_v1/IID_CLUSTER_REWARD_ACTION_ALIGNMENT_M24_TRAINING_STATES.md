# M24 reward action alignment

- Generated: 2026-07-30 05:08:12
- Commit: `9872c25a7d0243a40c67f5a360a2061ce706c046`
- Protocol: `iid-cluster-reward-action-alignment-m24-training-states-v1`
- Score: `lmb-cardinality-matched-iid-cluster-measurement-log-score`
- Training seeds: `[11 17 19 23 27 29]`
- Snapshot times: `[78 79 80 81 82 83]`
- Score truth used / target truth used: `0 / 1`
- Evidence boundary: The delayed score uses the exact LMB-induced measurement cardinality law and normalized measurement intensity, with no target truth. It is a log score for the IID-cluster projection, not the exact label-to-measurement association likelihood. Registered current-truth advantages are used only for alignment audit on already-opened training states. A pass may authorize a full observable reward dataset, but not bandit deployment, development evaluation, held-out M24, or X36.

| Metric | Value |
|:--|--:|
| Teacher actions | 144 |
| Runner-up actions | 144 |
| Teacher mean score advantage | 0.472504 |
| Teacher positive-score fraction | 0.5347 |
| Global score-task Spearman | 0.4384 |
| Pairwise preference accuracy | 0.6111 |
| Non-tie pairs | 144 |
| Positive per-seed Spearman fraction | 1.0000 |

| Seed | Score-task Spearman |
|--:|--:|
| 11 | 0.6812 |
| 17 | 0.1735 |
| 19 | 0.1847 |
| 23 | 0.6005 |
| 27 | 0.1349 |
| 29 | 0.4351 |

- Gates (row count / teacher positive / global rank / pairwise preference / per-seed rank / provenance): `1 / 0 / 1 / 1 / 1 / 1`
- All gates passed: `0`
- Full action reward dataset authorized: `0`
- Bandit / development / held-out M24 / X36 authorized: `0 / 0 / 0 / 0`
