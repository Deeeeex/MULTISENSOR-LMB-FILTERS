# V214 direct-graph formation withholding H=3

- Split / scene / seed / time: `training / x36-formation-fov / 1301 / 50`
- Any multi-objective eligible candidate: `0`
- Online posterior counterfactuals: `0`
- Moment-matched light posterior: `0`

| Formation action | E-OSPA gain | RMSE gain | Consensus gain | Terminal consensus | Byte saving | Worst sensor E / R | Minimum formation E / R | Eligible |
|:--|--:|--:|--:|--:|--:|--:|--:|:--:|
| v214-withhold-full-posterior-f1-one-page | -0.001% | -0.352% | +0.010% | +0.014% | +0.277% | +0.000 / +0.000% | -0.007 / -2.087% | 0 |
| v214-withhold-full-posterior-f2-one-page | +0.000% | +0.000% | +0.000% | +0.000% | +0.800% | +0.000 / +0.000% | +0.000 / +0.000% | 0 |
| v214-withhold-full-posterior-f3-one-page | +0.061% | -0.242% | -0.380% | -0.907% | +0.529% | +0.000 / +0.000% | +0.000 / -1.345% | 0 |
| v214-withhold-full-posterior-f4-one-page | +0.086% | -0.591% | -0.075% | -0.214% | -1.092% | +0.000 / +0.000% | +0.000 / -4.397% | 0 |
| v214-withhold-full-posterior-f5-one-page | +0.086% | -0.452% | +0.197% | +0.023% | +2.424% | +0.000 / -2.028% | +0.000 / -2.109% | 0 |
| v214-withhold-full-posterior-f6-one-page | +0.022% | -2.119% | +0.199% | -0.014% | +0.584% | +0.000 / +0.000% | +0.000 / -13.699% | 0 |

## Evidence boundary

This training-split H=3 screen starts every arm from the same truth-free V214 cache. One receiver formation withholds complete cross-formation posterior input for one page, then the full-payload reference resumes for two pages. Truth scores the return only. This is action-value training evidence, not an online policy, held-out result or generalization claim.
