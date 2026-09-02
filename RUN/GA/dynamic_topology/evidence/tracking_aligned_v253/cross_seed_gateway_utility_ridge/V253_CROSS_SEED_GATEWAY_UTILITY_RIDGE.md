# V253 cross-seed gateway utility ridge

- Analysis source commit: `79ddb0667adc21bf087c0c0a81bf273504b91ab2`
- V252 source commits: `e5cb7d2ddee17b568ad7e2a68004ff7492bfb4b2`
- Selected feature set: `payload-aware-328` (328 features)
- Selected lambda / activation threshold: `1 / 0.200`
- Training configurations compared: `100`
- Development holdout pass: `0`
- Next decision: `revise-observable-utility-representation-or-stop-before-gnn`

## Deployable objective

The ridge predicts candidate-minus-reference robust utility `min(E, R, C, byte, weakest-formation-E + 2, weakest-formation-R + 2)`. The reference maps exactly to zero. A non-reference gateway is used only when its predicted utility exceeds the frozen threshold; otherwise the selector abstains.

## Training-seed model selection

| Fold | Safe-positive selections | Reference-worst improved | E | RMSE | Consistency | Bytes | Weakest formation E / R | Pass |
|:--|--:|--:|--:|--:|--:|--:|:--|:--:|
| leave-seed-1302-out | 1/6 | 2/6 | -0.001% | +0.029% | +0.006% | -0.969% | -0.048% / -0.232% | 0 |
| leave-seed-1303-out | 1/6 | 2/6 | +0.051% | +0.647% | +0.472% | -0.159% | -0.022% / -0.008% | 0 |

## Seed-1304 frozen development holdout

| Anchor | Selected | Type | Predicted / realized utility | E / RMSE / C / bytes | Weakest formation E / R | Reference-worst E / R | Safe positive |
|--:|--:|:--|:--|:--|:--|:--|:--:|
| 40 | 3 | global-rank-profile | +23.725 / -1.525 | +0.147% / -0.440% / -0.001% / -1.427% | -0.000% / -3.525% | +0.001% / +1.495% | 0 |
| 60 | 3 | global-rank-profile | +51.065 / -0.984 | -0.071% / +8.999% / -0.984% / -0.726% | -0.936% / +0.983% | -0.936% / +18.864% | 0 |
| 80 | 12 | receiver-local-assignment | +10.850 / -0.226 | -0.226% / +0.450% / +0.268% / +0.011% | -1.064% / -0.000% | -0.000% / -0.000% | 0 |
| 100 | 20 | single-directed-arc | +13.391 / -0.891 | +0.045% / +3.252% / +1.363% / -0.891% | -0.658% / -0.481% | +0.717% / +6.012% | 0 |
| 120 | 20 | single-directed-arc | +31.737 / -0.714 | -0.184% / -0.714% / +0.002% / +0.272% | -0.333% / -0.913% | -0.178% / -0.913% | 0 |
| 140 | 13 | receiver-local-assignment | +4.353 / -0.001 | +0.000% / +0.005% / -0.001% / +0.084% | +0.000% / +0.000% | +0.000% / +0.001% | 0 |

Holdout aggregate: E `-0.048%`, RMSE `+1.524%`, consistency `+0.126%`, attempted-byte saving `-0.445%`, weakest formation E/R `-0.104% / +0.188%`; safe-positive selections `0/6`; gate `0`.

## Evidence boundary

V253 uses only V252 seeds 1302 and 1303 for feature-set, ridge-lambda and activation-threshold selection. Seed 1304 is evaluated exactly once after that choice is frozen. The scalar training target is the minimum slack across network E-OSPA, RMSE, consistency, attempted bytes and the two formation-tail tolerances. Features are current posterior/link/geometry/history summaries only; truth and future outcomes are labels, never inputs. Passing authorizes only a frozen complete-episode M24 study on untouched seed 1305, not a GNN, X36 or paper claim.
