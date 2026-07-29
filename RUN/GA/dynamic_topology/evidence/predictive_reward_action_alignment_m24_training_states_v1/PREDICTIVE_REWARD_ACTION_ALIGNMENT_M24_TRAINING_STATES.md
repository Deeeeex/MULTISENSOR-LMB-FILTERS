# M24 predictive reward action alignment

- Generated: 2026-07-30 04:27:50
- Commit: `1f5d6a693d5d5ffb54e9c3cb974db8f59a6f7406`
- Protocol: `predictive-reward-action-alignment-m24-training-states-v1`
- Training seeds: `[11 17 19 23 27 29]`
- Snapshot times: `[78 79 80 81 82 83]`
- Score truth used / target truth used: `0 / 1`
- Evidence boundary: The reward uses next measurements and no target truth. Registered current-truth task advantages are read only to audit alignment on already-opened training states. A pass may authorize a full observable reward dataset, but not bandit deployment, development evaluation, held-out M24, or X36.

| Metric | Value |
|:--|--:|
| Teacher actions | 144 |
| Runner-up actions | 144 |
| Teacher mean score advantage | 0.430689 |
| Teacher positive-score fraction | 0.5417 |
| Global score-task Spearman | 0.4196 |
| Pairwise preference accuracy | 0.5903 |
| Non-tie pairs | 144 |
| Positive per-seed Spearman fraction | 1.0000 |

| Seed | Score-task Spearman |
|--:|--:|
| 11 | 0.6971 |
| 17 | 0.1365 |
| 19 | 0.1412 |
| 23 | 0.5648 |
| 27 | 0.0883 |
| 29 | 0.4132 |

- Gates (row count / teacher positive / global rank / pairwise preference / per-seed rank / provenance): `1 / 0 / 1 / 0 / 1 / 1`
- All gates passed: `0`
- Full action reward dataset authorized: `0`
- Bandit / development / held-out M24 / X36 authorized: `0 / 0 / 0 / 0`
