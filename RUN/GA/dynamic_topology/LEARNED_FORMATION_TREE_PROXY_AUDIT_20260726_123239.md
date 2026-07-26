# Learned formation-tree proxy audit

- Generated: 2026-07-26 12:32:39
- Source weight: 0.70
- Baseline mode: `fixed-index-star`
- Feature context: `raw`
- Baseline phase: 1
- Model family: `knn`
- Selected hyperparameter: 10
- Training blocks: `[1 2 3 4 5 6]`
- Leave-one-seed-out CV seeds: `[7 11]`
- Validation blocks: `[7 8 9]`
- Training gate passed: `0`
- Validation gate passed: `0`
- All observed blocks pass: `0`
- Evidence boundary: The edge model uses privileged labels only on declared training blocks. Projection-time features are truth-free. Validation labels do not select the model. This is not closed-loop evidence.

## Cross-validated model selection

| Hyperparameter | Worst fold gain | Mean fold gain | Worst harmful | Worst selected residual |
|--:|--:|--:|--:|--:|
| 10 | -0.0476 | 0.0374 | 1.0000 | -0.6018 |
| 5 | -0.0676 | 0.0322 | 1.0000 | -0.5688 |
| 20 | -0.0701 | -0.0037 | 1.0000 | -0.8090 |
| 80 | -0.0904 | -0.0197 | 1.0000 | -0.8439 |
| 40 | -0.1469 | -0.0263 | 1.0000 | -3.6073 |

## Frozen-model projection audit

| Block | Split | Risk improvement vs baseline | Privileged projected-tree gain | Worst selected edge | Harmful edges | Bytes ratio | Root | Formation parents |
|:--|:--|--:|--:|--:|--:|--:|--:|:--|
| m24-hard seed 7 t=75 | train | -0.0114 | 0.0727 | -0.5848 | 0.6667 | 1.0029 | 1 | `[0 1 2 3]` |
| m24-hard seed 7 t=76 | train | 0.0199 | 0.1606 | -0.1769 | 0.6667 | 1.0064 | 1 | `[0 3 1 2]` |
| m24-hard seed 7 t=77 | train | 0.0087 | 0.0732 | -0.5961 | 0.6667 | 1.0034 | 1 | `[0 1 4 2]` |
| m24-hard seed 11 t=75 | train | 0.1090 | 0.1590 | 0.0323 | 0.0000 | 1.0022 | 2 | `[2 0 2 3]` |
| m24-hard seed 11 t=76 | train | 0.1724 | 0.1766 | 1.4577 | 0.0000 | 0.9958 | 1 | `[0 1 2 3]` |
| m24-hard seed 11 t=77 | train | 0.0979 | 0.1353 | 0.0344 | 0.0000 | 0.9959 | 1 | `[0 3 1 3]` |
| m24-hard seed 17 t=75 | validation | -0.0897 | 0.0993 | -1.6771 | 0.3333 | 1.0046 | 4 | `[4 3 4 0]` |
| m24-hard seed 17 t=76 | validation | -0.0232 | 0.1157 | -0.4817 | 1.0000 | 0.9933 | 2 | `[2 0 2 3]` |
| m24-hard seed 17 t=77 | validation | -0.0772 | 0.0535 | -2.0232 | 0.6667 | 0.9701 | 1 | `[0 1 2 3]` |

## Selected sensor edges

- m24-hard seed 7: receivers `[12 18 23]`, senders `[2 8 13]`, predicted `[0.42341;0.55503;0.14916]`, actual `[0.57705;-0.022181;-0.58484]`
- m24-hard seed 7: receivers `[10 17 24]`, senders `[17 2 9]`, predicted `[2.1544;0.2459;0.04295]`, actual `[-0.062822;1.0423;-0.17687]`
- m24-hard seed 7: receivers `[11 17 22]`, senders `[1 24 8]`, predicted `[0.069138;0.37231;-0.060574]`, actual `[-0.59611;0.69265;-0.071015]`
- m24-hard seed 11: receivers `[1 17 22]`, senders `[10 8 13]`, predicted `[0.7137;1.6294;2.7402]`, actual `[0.032269;1.7592;1.2762]`
- m24-hard seed 11: receivers `[11 17 23]`, senders `[5 7 14]`, predicted `[0.87966;3.0089;1.495]`, actual `[1.8255;4.1923;1.4577]`
- m24-hard seed 11: receivers `[10 17 23]`, senders `[17 3 13]`, predicted `[1.3474;1.4393;0.11879]`, actual `[0.034386;2.9562;0.46989]`
- m24-hard seed 17: receivers `[5 7 16]`, senders `[19 16 23]`, predicted `[0.21155;0.5009;2.6881]`, actual `[-1.6771;0.0076828;0.29938]`
- m24-hard seed 17: receivers `[6 18 23]`, senders `[10 12 13]`, predicted `[0.19059;3.2942;2.8581]`, actual `[-0.081124;-0.48171;-0.18331]`
- m24-hard seed 17: receivers `[7 17 20]`, senders `[5 12 18]`, predicted `[0.16449;3.246;2.6141]`, actual `[-2.0232;0.024707;-0.14895]`
