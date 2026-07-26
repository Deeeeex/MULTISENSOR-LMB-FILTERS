# Learned formation-tree proxy audit

- Generated: 2026-07-26 12:32:53
- Source weight: 0.70
- Baseline mode: `fixed-index-star`
- Feature context: `graph-context`
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
| 10 | -0.0525 | 0.0127 | 1.0000 | -0.6823 |
| 20 | -0.0583 | -0.0184 | 1.0000 | -1.5742 |
| 40 | -0.0618 | -0.0241 | 1.0000 | -1.4336 |
| 5 | -0.0739 | 0.0089 | 1.0000 | -1.4336 |
| 80 | -0.0904 | -0.0134 | 1.0000 | -1.4336 |

## Frozen-model projection audit

| Block | Split | Risk improvement vs baseline | Privileged projected-tree gain | Worst selected edge | Harmful edges | Bytes ratio | Root | Formation parents |
|:--|:--|--:|--:|--:|--:|--:|--:|:--|
| m24-hard seed 7 t=75 | train | -0.0309 | 0.0727 | -0.6098 | 0.6667 | 1.0039 | 1 | `[0 1 2 3]` |
| m24-hard seed 7 t=76 | train | 0.0413 | 0.1606 | -0.0628 | 0.6667 | 1.0041 | 1 | `[0 3 1 2]` |
| m24-hard seed 7 t=77 | train | 0.0107 | 0.0732 | -0.6375 | 0.3333 | 1.0039 | 1 | `[0 1 4 2]` |
| m24-hard seed 11 t=75 | train | 0.1301 | 0.1590 | 1.0507 | 0.0000 | 1.0006 | 1 | `[0 1 2 3]` |
| m24-hard seed 11 t=76 | train | 0.1733 | 0.1766 | 1.4577 | 0.0000 | 0.9964 | 1 | `[0 1 2 3]` |
| m24-hard seed 11 t=77 | train | 0.0715 | 0.1353 | -0.6209 | 0.6667 | 0.9970 | 1 | `[0 1 2 3]` |
| m24-hard seed 17 t=75 | validation | -0.1202 | 0.0993 | -1.6771 | 0.6667 | 1.0075 | 3 | `[4 3 0 3]` |
| m24-hard seed 17 t=76 | validation | -0.0508 | 0.1157 | -0.4948 | 1.0000 | 0.9949 | 3 | `[2 3 0 3]` |
| m24-hard seed 17 t=77 | validation | -0.0391 | 0.0535 | -0.3711 | 0.6667 | 0.9987 | 4 | `[2 4 2 0]` |

## Selected sensor edges

- m24-hard seed 7: receivers `[11 18 19]`, senders `[4 7 13]`, predicted `[0.39123;0.24029;0.17045]`, actual `[0.59023;-0.60177;-0.60976]`
- m24-hard seed 7: receivers `[10 17 24]`, senders `[17 5 8]`, predicted `[1.8235;0.066586;1.1831]`, actual `[-0.062822;1.6584;-0.058364]`
- m24-hard seed 7: receivers `[11 17 21]`, senders `[3 24 12]`, predicted `[0.065163;0.44681;-0.032824]`, actual `[-0.63753;0.69265;0.0098427]`
- m24-hard seed 11: receivers `[11 17 22]`, senders `[6 7 13]`, predicted `[0.31422;1.3388;1.9019]`, actual `[1.9607;1.0507;1.2762]`
- m24-hard seed 11: receivers `[11 17 23]`, senders `[5 8 14]`, predicted `[0.83879;3.0703;0.66192]`, actual `[1.8255;4.2339;1.4577]`
- m24-hard seed 11: receivers `[11 17 21]`, senders `[3 7 13]`, predicted `[0.1021;3.0032;0.23468]`, actual `[-0.62092;4.942;-0.14472]`
- m24-hard seed 17: receivers `[5 7 24]`, senders `[19 13 14]`, predicted `[0.11672;2.3749;2.1483]`, actual `[-1.6771;-0.55664;0.22495]`
- m24-hard seed 17: receivers `[3 8 23]`, senders `[10 17 14]`, predicted `[0.16239;2.2249;3.2792]`, actual `[-0.49481;-0.05813;-0.059156]`
- m24-hard seed 17: receivers `[6 12 17]`, senders `[9 19 8]`, predicted `[0.23283;1.8617;3.1182]`, actual `[-0.37109;0.30412;-0.10007]`
