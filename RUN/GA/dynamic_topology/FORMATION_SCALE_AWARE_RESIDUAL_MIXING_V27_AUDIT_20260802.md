# V27 scale-aware residual mixing audit

## Decision

**Reject global formation-mixing acceleration as the next method direction.** The frozen primary screen found `0/10` strong nonreference actions. No additional v27 M24 state, X36/X48 outcome, or GNN training is authorized.

- Contract: `formation-scale-aware-mixing-v27-mechanism-audit-v1`
- Source screen: `RUN/GA/dynamic_topology/evidence/formation_value_v27/scale_aware_residual_mixing/t72/screen/FORMATION_SCALE_AWARE_MIXING_H3_M24_FORMATION_FOV_SEED211_T72.mat`
- Generation commit: `9d9fc7e003a868f688718061aac5c01eebb0b78d`
- Cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Preset / seed / time: `m24-formation-fov / 211 / 72`
- Return times: `[72 73 74]`
- New tracking state opened by this audit: `0`

## What the intervention shows

Actions 1, 2, and 3 have exactly the same directed adjacency and the same 40-message budget. Only the cross-formation residual weight changes from 0.05 to 0.10 and 0.15. Therefore the losses of actions 2 and 3 cannot be attributed to adding edges, changing message count, or selecting a different gateway graph.

| Action | Same adjacency | Max weight change | Mix gap | Mean gain | Window consensus gain | First-step gain |
|:--|:--:|--:|--:|--:|--:|--:|
| `reference-cycle-cross-w0p05` | 1 | 0.000 | 0.008298 | +0.000% | +0.000% | +0.000% |
| `cycle-cross-w0p10` | 1 | 0.050 | 0.016525 | -1.318% | -4.841% | -0.975% |
| `cycle-cross-w0p15` | 1 | 0.100 | 0.024680 | -3.170% | -11.899% | -2.321% |

## Agreement is not correctness

Every nonreference action lowers the moment-summary posterior disagreement, yet every one increases mean cardinality error. The sensors become more similar to each other while becoming jointly worse at deciding how many labels exist.

| Action | Posterior disagreement | Position disagreement | Mean cardinality error | Cardinality dispersion | Delivered bytes |
|:--|--:|--:|--:|--:|--:|
| `reference-cycle-cross-w0p05` | 1.934789 | 14.348088 | 2.708333 | 0.819444 | 5277944 |
| `cycle-cross-w0p10` | 1.909203 | 14.305976 | 2.819444 | 0.930556 | 5181608 |
| `cycle-cross-w0p15` | 1.882840 | 14.268271 | 2.972222 | 1.083333 | 5174168 |
| `dual-gateway-cycle-cross-w0p05` | 1.815515 | 13.811092 | 3.166667 | 1.250000 | 5090528 |
| `dual-gateway-cycle-cross-w0p10` | 1.763438 | 18.269560 | 3.486111 | 1.569444 | 4981376 |
| `bidirectional-cycle-cross-w0p05` | 1.835213 | 14.164897 | 3.055556 | 1.138889 | 5176320 |
| `bidirectional-cycle-cross-w0p10` | 1.811683 | 13.962818 | 3.194444 | 1.277778 | 5058576 |
| `cycle-antipodal-cross-w0p05` | 1.795200 | 13.437657 | 3.250000 | 1.333333 | 5097232 |
| `cycle-antipodal-cross-w0p10` | 1.747583 | 13.768262 | 3.527778 | 1.611111 | 4964536 |
| `bidirectional-antipodal-cross-w0p05` | 1.681990 | 13.825092 | 3.666667 | 1.583333 | 5107728 |
| `bidirectional-antipodal-cross-w0p10` | 1.621148 | 13.577173 | 4.013889 | 1.958333 | 4934520 |

- Posterior-agreement/cardinality split: `10/10` nonreference actions.
- Mixing gap versus mean tracking loss, Pearson `r = 0.824711`.
- Mixing gap versus window-consensus loss, Pearson `r = 0.826776`.
These correlations are diagnostic associations within one opened state, not generalization claims.

## Why the existing posterior-risk score misses it

For a Bernoulli label, the current proxy is

`min(r, (1-r) + r v)`, where `v = min(trace(P)/c^2, 1)`.

When KLA conflict suppresses `r`, the empty-set term `r` also shrinks. The score can therefore reward deleting a reference-supported label. It measures internal decision confidence, not retention of plausible label existence. The implementation itself correctly states that confidently biased posteriors are not detectable; v27 shows that a disagreement guard does not repair this blind spot because agreement can improve while all nodes move toward the same wrong cardinality.

## KLA mechanism and v28 requirement

For label `ell`, Bernoulli KLA gives

`logit(r_bar_ell) = sum_i omega_i logit(r_i,ell) + log(eta_ell)`,

with `eta_ell = integral prod_i p_i,ell(x)^omega_i dx <= 1`. Spatial conflict makes `log(eta_ell)` negative and can suppress existence below pruning or MAP thresholds. Stronger mixing is therefore beneficial only when its contraction gain exceeds this endogenous conflict disturbance.

V28 must keep a hard connectivity floor but change the action from global acceleration to **conflict-aware selective mixing**. Each candidate must be compared with the reference using current posteriors and must pass label-existence-retention/cardinality-risk constraints before any disagreement or spectral-gap objective can rank it. Extra cross trust may be assigned only to compatible or complementary formation pairs; incompatible pairs must retain or reduce reference trust.

## Full frozen outcome table

| Action | Mix gap | Mean gain | Worst gain | Min formation | Window consensus | Final consensus | Byte saving | Strong |
|:--|--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-cycle-cross-w0p05` | 0.008298 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 0 |
| `cycle-cross-w0p10` | 0.016525 | -1.318% | -11.151% | -3.038% | -4.841% | -3.910% | +1.746% | 0 |
| `cycle-cross-w0p15` | 0.024680 | -3.170% | -21.322% | -6.818% | -11.899% | -12.964% | +1.880% | 0 |
| `dual-gateway-cycle-cross-w0p05` | 0.016525 | -6.848% | -2.282% | -12.218% | -20.863% | -12.144% | +1.298% | 0 |
| `dual-gateway-cycle-cross-w0p10` | 0.032759 | -10.999% | -7.049% | -15.995% | -34.220% | -17.613% | +3.567% | 0 |
| `bidirectional-cycle-cross-w0p05` | 0.016667 | -5.972% | +5.212% | -13.305% | -16.209% | -14.472% | +1.302% | 0 |
| `bidirectional-cycle-cross-w0p10` | 0.033333 | -7.325% | -8.863% | -17.104% | -22.010% | -16.402% | +3.423% | 0 |
| `cycle-antipodal-cross-w0p05` | 0.016667 | -8.358% | +4.870% | -10.285% | -24.621% | -21.983% | +1.902% | 0 |
| `cycle-antipodal-cross-w0p10` | 0.033333 | -11.465% | -12.151% | -13.635% | -35.083% | -22.715% | +4.490% | 0 |
| `bidirectional-antipodal-cross-w0p05` | 0.033333 | -15.442% | -1.977% | -25.277% | -37.008% | -39.804% | +2.656% | 0 |
| `bidirectional-antipodal-cross-w0p10` | 0.066667 | -19.433% | -10.878% | -31.047% | -50.543% | -52.671% | +5.966% | 0 |

## Evidence boundary

This deterministic audit reuses one already-opened M24 state. Outcome correlations diagnose a mechanism but do not establish generalization. The identical-adjacency weight intervention is stronger within-state evidence, yet fresh multi-state gates are still required before training or validation.
