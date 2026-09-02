# V254 compact additive gateway ridge

- Analysis source commit: `1f6f39f997d98464716d28790eab860fdfbfc61f`
- V252 source commits: `e5cb7d2ddee17b568ad7e2a68004ff7492bfb4b2`
- Feature set: `compact-additive-16` (16 features)
- Control synopsis: `compact-node-32`
- Lambda / activation threshold: `1 / 0.200`
- Candidate-bank holdout pass: `0`
- Applied exact-projection bank coverage: `0/5`
- Next decision: `revise-additive-observable-representation-or-stop-before-gnn`

## Deployable objective and accounting

The ridge predicts candidate-minus-V242 robust utility from the sum of 16 compact edge contributions. Each evaluated policy window adds its fixed 32-byte-per-sensor synopsis and route command to attempted bytes, including windows that abstain. The V242 comparator pays no learned-policy control overhead.

## Training-seed model selection

| Fold | Safe positive | E | RMSE | Consistency | Total bytes | Weakest formation E / R | Pass |
|:--|--:|--:|--:|--:|--:|:--|:--:|
| leave-seed-1302-out | 0/6 | +0.037% | -1.855% | +0.003% | -0.398% | +0.000% / -3.290% | 0 |
| leave-seed-1303-out | 0/6 | -0.024% | +0.103% | -0.030% | -1.007% | -0.050% / -0.020% | 0 |

## Seed-1304 frozen candidate-bank holdout

| Anchor | Selected | Type | Predicted / realized utility | E / RMSE / C / total bytes | Control B | Safe positive |
|--:|--:|:--|:--|:--|--:|:--:|
| 40 | 20 | single-directed-arc | +1.683 / -2.188 | +0.018% / +0.046% / -0.526% / -2.188% | 2496 | 0 |
| 60 | 9 | receiver-local-assignment | +3.168 / -0.981 | -0.157% / +6.842% / -0.981% / -0.184% | 2496 | 0 |
| 80 | 22 | single-directed-arc | +1.027 / -2.174 | -0.065% / +7.811% / -1.014% / -2.174% | 2496 | 0 |
| 100 | 7 | receiver-local-assignment | +2.005 / -1.485 | +0.000% / +0.011% / +0.000% / -1.485% | 2496 | 0 |
| 120 | 1 | v242-reference | +0.000 / +0.000 | +0.000% / +0.000% / +0.000% / -0.358% | 2496 | 0 |
| 140 | 1 | v242-reference | +0.000 / +0.000 | +0.000% / +0.000% / +0.000% / -0.435% | 2496 | 0 |

Holdout aggregate: E `-0.033%`, RMSE `+1.644%`, consistency `-0.409%`, total-byte saving `-1.152%`, weakest formation E/R `-0.094% / -0.000%`; posterior/control/total attempted bytes `3797976 / 14976 / 3812952`; safe-positive selections `0/6`; gate `0`.

## Exact projection coverage

| Anchor | Applied | Predicted advantage | Changed arcs | Projection fallback | V252 bank index |
|--:|:--:|--:|--:|:--:|--:|
| 40 | 1 | +5.033 | 4 | 0 | 0 |
| 60 | 1 | +4.904 | 4 | 0 | 0 |
| 80 | 1 | +1.584 | 1 | 0 | 0 |
| 100 | 1 | +3.895 | 2 | 0 | 0 |
| 120 | 1 | +1.651 | 2 | 0 | 0 |
| 140 | 0 | +0.000 | 0 | 0 | 1 |

This diagnostic observes only which exact projected assignments fall inside the frozen V252 bank. Out-of-bank actions have no tracking outcome until a separate paired H=3 continuation is run.

## Evidence boundary

V254 fits one reference-centered additive ridge on V252 seeds 1302 and 1303. Hyperparameters are chosen by leave-one-training-seed-out evaluation before seed 1304 is evaluated once. Only the fixed 32-byte-per-sensor compact telemetry contract is eligible for the primary model. Every selected or fallback window is charged the compact synopsis and route-command bytes, while the V242 comparator pays no learned-policy overhead. Candidate-bank holdout performance may authorize a frozen H=3 evaluation of the exact projected action; it does not by itself authorize a complete episode, X36, a GNN or a paper claim.
