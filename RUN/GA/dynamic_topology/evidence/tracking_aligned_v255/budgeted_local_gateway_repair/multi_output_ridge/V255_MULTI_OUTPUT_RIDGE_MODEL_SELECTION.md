# V255 multi-output ridge model selection

- Analysis source commit: `e97cbde6d12548d42ed7738d7304397f228eba58`
- V252 source commits: `e5cb7d2ddee17b568ad7e2a68004ff7492bfb4b2`
- Action features / outcomes: `32 / 8`
- Ridge lambda: `1`
- Receiver-RMSE activation threshold: `1.000%`
- Passing held-seed folds: `1/3`
- Cross-seed gate: `0`
- Next decision: `revise-local-observable-features-before-new-holdout`

The model contains one ridge member per training seed. Each outcome coordinate uses the minimum prediction across members; an action is accepted only when all registered lower-envelope constraints pass. The selected objective is receiving-formation RMSE gain, while network RMSE remains a positive constraint.

| Fold | Safe positive | E | RMSE | Consistency | Total bytes | Weakest formation E / R | Mean receiver RMSE | Pass |
|:--|--:|--:|--:|--:|--:|:--|--:|:--:|
| leave-seed-1302-out | 0/6 | +0.000% | +0.000% | +0.000% | -0.067% | +0.000% / +0.000% | +0.000% | 0 |
| leave-seed-1303-out | 0/6 | +0.000% | +0.000% | +0.000% | -0.069% | +0.000% / +0.000% | +0.000% | 0 |
| leave-seed-1304-out | 1/6 | +0.000% | +0.013% | +0.000% | -0.253% | +0.000% / +0.000% | +0.191% | 1 |

Cross-seed aggregate: E `+0.000%`, RMSE `+0.006%`, consistency `+0.000%`, total-byte saving `-0.131%`, weakest formation E/R `+0.000% / +0.000%`; safe-positive selections `1/18`; gate `0`.

## Per-window held-seed decisions

| Fold | Anchor | Selected | Lower receiver RMSE | Realized E / R / C / B / receiver R | Safe |
|:--|--:|--:|--:|:--|:--:|
| leave-seed-1302-out | 40 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1302-out | 60 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1302-out | 80 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1302-out | 100 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1302-out | 120 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1302-out | 140 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1303-out | 40 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1303-out | 60 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1303-out | 80 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1303-out | 100 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1303-out | 120 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1303-out | 140 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1304-out | 40 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1304-out | 60 | 11 | +1.175% | +0.000 / +0.142 / +0.000 / -1.245 / +1.145 | 1 |
| leave-seed-1304-out | 80 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1304-out | 100 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1304-out | 120 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |
| leave-seed-1304-out | 140 | 1 | +0.000% | +0.000 / +0.000 / +0.000 / +0.000 / +0.000 | 0 |

## Evidence boundary

V255 is a development-stage repair of the V242 minimum causal backbone. One action may replace at most one directed physical gateway while preserving the formation tree, local cycles, KLA weights and N+2(F-1) posterior-message count. Controller telemetry is collected once per H=3 hold and only from the source and receiver formations. The learned model predicts eight network, tail and receiver-formation outcome coordinates separately; a scalar minimum-slack regression is not reused. Seeds 1302--1304 are development data, seed 1306 is a new frozen holdout, and seed 1305 remains untouched for a later complete episode. Topology and communication-credit constraints are deterministic; tracking gains require paired evidence and do not authorize a GNN, X36 or a paper claim.
