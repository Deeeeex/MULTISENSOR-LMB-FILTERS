# M24 label-set representation learnability

- Generated: 2026-07-30 06:02:23
- Commit: `a0f081e2c133701afb7189a2c02c290b38295e48`
- Protocol: `m24-label-set-simulator-policy-redesign-v1`
- Seeds: `[11 17 19 23 27 29]`
- States / pairs: `36 / 144`
- Valid edge-label rows: `165771`
- Maximum labels per edge: `16`
- Evidence boundary: All model inputs are truth-free observable posterior, link and history features. Current tracking truth trains this preflight only to test whether per-label structure transfers across whole M24 training seeds. It is not a deployable policy, an imitation claim or a closed-loop result. Even a pass authorizes only the message-passing implementation; policy selection must use paired closed-loop simulator return. Development M24, held-out M24 and X36 remain sealed.

| Whole-seed outer metric | Aggregate task ridge | Label-set DeepSets |
|:--|--:|--:|
| Pairwise preference accuracy | 0.6250 | 0.7083 |
| Difference Spearman | 0.2064 | 0.4625 |
| Mean absolute difference error | 0.058004 | 0.037014 |

- Pairwise accuracy improvement: `0.0833`
- Difference Spearman improvement: `0.2561`
- Positive per-seed label-set Spearman fraction: `1.0000`

| Held-out seed | Aggregate acc. | Aggregate rho | Label-set acc. | Label-set rho |
|--:|--:|--:|--:|--:|
| 11 | 0.7083 | 0.3504 | 0.7083 | 0.5783 |
| 17 | 0.5417 | 0.0470 | 0.7500 | 0.3861 |
| 19 | 0.6667 | 0.2252 | 0.8333 | 0.5991 |
| 23 | 0.6667 | 0.1113 | 0.7083 | 0.3904 |
| 27 | 0.5833 | -0.0287 | 0.7083 | 0.1896 |
| 29 | 0.5833 | 0.3600 | 0.5417 | 0.3052 |

- Gates (source / preference / rank / per-seed rank / aggregate improvement): `1 / 1 / 1 / 1 / 1`
- All gates passed: `1`
- Label-set representation accepted: `1`
- Message-passing implementation authorized: `1`
- Closed-loop policy / development / held-out M24 / X36 authorized: `0 / 0 / 0 / 0`
