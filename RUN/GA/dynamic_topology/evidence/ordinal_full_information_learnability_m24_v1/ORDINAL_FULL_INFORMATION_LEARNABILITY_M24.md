# M24 ordinal full-information learnability

- Generated: 2026-07-30 05:20:13
- Commit: `25579cd448425b825296432745cdd513106092f6`
- Protocol: `iid-cluster-ordinal-full-information-learnability-m24-v1`
- Seeds: `[11 17 19 23 27 29]`
- Pair count: `144`
- Evidence boundary: Outer and inner splits hold out complete scenario seeds. Model fitting and hyperparameter selection use only observable IID-cluster score differences and truth-free graph features. Current tracking truth is used only for the secondary outer task alignment readout. A pass authorizes only an expanded observable reward dataset, not policy deployment or any sealed split.

| Outer metric | Value |
|:--|--:|
| Surrogate pairwise accuracy | 0.5069 |
| Surrogate difference Spearman | 0.0479 |
| Positive per-seed surrogate Spearman | 0.3333 |
| Task preference accuracy | 0.5764 |
| Task difference Spearman | 0.1222 |
| Mean selected task advantage | 0.065165 |
| Minimum selected task advantage | -0.149289 |

| Held-out seed | Variant | Lambda | Surrogate acc. | Surrogate rho | Task acc. | Task rho |
|--:|:--|--:|--:|--:|--:|--:|
| 11 | edge-only | 10 | 0.4583 | -0.0191 | 0.5000 | -0.1991 |
| 17 | edge-only | 1 | 0.2917 | -0.2209 | 0.3750 | -0.1765 |
| 19 | edge-only | 0.1 | 0.5833 | 0.4017 | 0.7917 | 0.5487 |
| 23 | edge-only | 0.01 | 0.7500 | 0.3078 | 0.5417 | 0.1574 |
| 27 | edge-receiver-sender-difference | 100 | 0.4583 | -0.4435 | 0.6250 | 0.2113 |
| 29 | edge-only | 0.01 | 0.5000 | -0.0017 | 0.6250 | -0.0443 |

- Gates (source / surrogate preference / surrogate rank / per-seed rank / task preference / selected task advantage): `1 / 0 / 0 / 0 / 1 / 1`
- All gates passed: `0`
- Expanded reward dataset authorized: `0`
- Bandit / development / held-out M24 / X36 authorized: `0 / 0 / 0 / 0`
