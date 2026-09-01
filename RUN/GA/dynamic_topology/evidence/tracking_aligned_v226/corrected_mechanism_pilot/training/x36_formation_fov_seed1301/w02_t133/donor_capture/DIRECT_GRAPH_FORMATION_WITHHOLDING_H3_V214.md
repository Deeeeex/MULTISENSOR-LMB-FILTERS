# V214 direct-graph formation withholding H=3

- Split / scene / seed / time: `training / x36-formation-fov / 1301 / 133`
- Any multi-objective eligible candidate: `1`
- Best candidate: `v214-withhold-full-posterior-f1-one-page`
- Online posterior counterfactuals: `0`
- Moment-matched light posterior: `0`

| Formation action | E-OSPA gain | RMSE gain | Consensus gain | Terminal consensus | Byte saving | Worst sensor E / R | Minimum formation E / R | Eligible |
|:--|--:|--:|--:|--:|--:|--:|--:|:--:|
| v214-withhold-full-posterior-f1-one-page | +0.184% | +0.465% | +0.547% | +0.647% | +0.404% | +0.000 / +0.000% | +0.000 / +0.000% | 1 |
| v214-withhold-full-posterior-f2-one-page | -0.037% | -0.307% | +0.003% | -0.014% | +0.413% | +0.000 / +0.000% | -0.232 / -3.697% | 0 |
| v214-withhold-full-posterior-f3-one-page | -0.001% | -0.019% | -0.002% | -0.011% | +1.301% | +0.000 / +0.000% | -0.005 / -0.112% | 0 |
| v214-withhold-full-posterior-f4-one-page | -0.004% | -0.105% | -0.069% | -0.225% | +0.183% | +0.000 / +0.000% | -0.025 / -0.438% | 0 |
| v214-withhold-full-posterior-f5-one-page | +0.372% | +0.022% | +0.400% | -0.189% | +0.025% | +2.877 / +6.100% | +0.000 / +0.000% | 0 |
| v214-withhold-full-posterior-f6-one-page | -0.009% | -0.763% | +0.175% | +0.016% | +0.485% | +0.000 / +0.000% | -0.050 / -4.973% | 0 |

## Evidence boundary

This training-split H=3 screen starts every arm from the same truth-free V214 cache. One receiver formation withholds complete cross-formation posterior input for one page, then the full-payload reference resumes for two pages. Truth scores the return only. This is action-value training evidence, not an online policy, held-out result or generalization claim.
