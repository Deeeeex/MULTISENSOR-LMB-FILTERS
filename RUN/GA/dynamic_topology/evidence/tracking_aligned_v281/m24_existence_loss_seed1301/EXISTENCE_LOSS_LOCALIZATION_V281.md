# V281: one-round existence-loss localization

M24, opened seed 1301, saved pre-topology/pre-fusion local posteriors at t=70,84,151. Only the saved V242 reference gateway assignment is replayed. No recursive filter, new policy, truth-label matching or parameter scan.

The scheduled graph and directed delivery uniforms are regenerated; these are current-code one-round replays of cached local states, not new full-episode measurements or an X36 causal explanation.

## Stage means over 24 receivers per anchor

| Time | Local MAP count | Scheduled label union | Labels lost with packets | Actual input label union | Before-overlap existence mass | Fused existence mass | Retained labels | Output MAP count |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 70 | 5.958 | 15.500 | 0.000 | 15.500 | 6.042 | 5.923 | 14.958 | 6.000 |
| 84 | 5.917 | 15.917 | 0.000 | 15.917 | 6.001 | 5.776 | 15.333 | 5.833 |
| 151 | 6.542 | 15.417 | 0.000 | 15.417 | 6.592 | 6.493 | 14.750 | 6.667 |

## Per-formation mechanisms (six receivers per row)

| Time | Formation | No-input network labels | Weak before overlap | Cross below 0.5 after overlap | Existence mass lost to overlap | Mass pruned | Retained but not output mass | Powered-GM used / fused labels | Normalizer fallback labels | Output MAP min / median / max |
|--:|--:|--:|--:|--:|--:|--:|--:|:--|--:|:--|
| 70 | 1 | 1.000 | 7.167 | 0.167 | 0.188 | 0.00208 | 0.541 | 9 / 90 | 20 | 7 / 7.0 / 9 |
| 70 | 2 | 0.000 | 10.000 | 0.667 | 0.131 | 0.00000 | 1.091 | 3 / 96 | 9 | 5 / 5.5 / 7 |
| 70 | 3 | 0.167 | 10.833 | 0.000 | 0.125 | 0.00271 | 0.450 | 4 / 95 | 24 | 5 / 5.0 / 5 |
| 70 | 4 | 0.833 | 9.000 | 0.000 | 0.032 | 0.00084 | 0.399 | 2 / 91 | 25 | 5 / 6.0 / 7 |
| 84 | 1 | 0.333 | 8.833 | 0.333 | 0.411 | 0.00139 | 0.920 | 17 / 94 | 18 | 5 / 6.0 / 7 |
| 84 | 2 | 0.000 | 8.000 | 0.000 | 0.331 | 0.00000 | 0.863 | 21 / 96 | 3 | 6 / 7.0 / 7 |
| 84 | 3 | 0.000 | 11.667 | 0.167 | 0.101 | 0.00301 | 0.564 | 6 / 96 | 26 | 4 / 5.0 / 5 |
| 84 | 4 | 0.000 | 10.000 | 0.000 | 0.057 | 0.00248 | 0.517 | 0 / 96 | 31 | 6 / 6.0 / 6 |
| 151 | 1 | 0.667 | 13.333 | 0.000 | 0.066 | 0.00235 | 0.808 | 8 / 92 | 0 | 2 / 3.0 / 3 |
| 151 | 2 | 1.333 | 8.000 | 0.000 | 0.026 | 0.00425 | 0.449 | 0 / 88 | 24 | 6 / 6.5 / 7 |
| 151 | 3 | 0.000 | 5.500 | 0.000 | 0.137 | 0.00000 | 0.775 | 9 / 96 | 30 | 9 / 10.0 / 10 |
| 151 | 4 | 0.333 | 7.500 | 0.000 | 0.168 | 0.00939 | 0.629 | 13 / 94 | 21 | 7 / 8.0 / 8 |

## What this narrows down

Of 1124 fused label records, 659 already have negative weighted input log odds before the spatial term. Among those, 29 have at least one actual input with existence >=0.9; 0 include a censored-absence participant. These are descriptive thresholds, not an output rule.

The single-round spatial term removes mean existence mass 0.119/0.225/0.099 at the three anchors; pruning removes 0.00238 per receiver snapshot overall. The pre-fusion local MAP readout is already about six. This does not support making the final spatial term or output threshold the first intervention. Trace earlier local updates and propagation, while retaining the smaller input-weighting effect as a distinct mechanism. Earlier compounded spatial losses remain possible.


## Interpretation boundaries

- A label is a Bernoulli identifier, not a truth-matched target; network label unions can contain duplicates of physical targets and false tracks.
- Transport loss counts labels present in at least one scheduled payload but absent from self and all nonempty delivered payloads. An absent individual label inside a nonempty packet is handled by the unchanged FoV-censored fusion rule.
- Before-overlap mass sums sigmoid(weighted input log odds), including FoV-censored absence overrides. It is an algebraic diagnostic, not an alternative fusion method or expected true-target recall.
- The 0.5 crossing only locates changes in log-odds sign. Actual output uses the unchanged existence pruning threshold and lmbMapCardinalityEstimate, not a per-label 0.5 extractor. Unselected retained labels stay in memory.
- Powered-GM normalizer fallback is counted separately. The current fusion rejects powered-GM overlap greater than one and uses its moment-matched fallback; this readout does not measure approximation error against exact KLA.
- The cache is already after local updating. Earlier recursive transport, measurement/association and local-update losses cannot be attributed by these anchors.

Collected 72 receiver snapshots; 4 lost incoming packets and 0 delivered empty inputs. Maximum probability-form existence identity residual: 2.22e-16. This is producing-agent self-check only.

## Sources

- Runtime source commit: `764de2c7236e0f6eed9ac248730f7f3aa650da82`
- Cache source commit: `3248df0edeba8e8e24d897821a4922522cdec5a8`
- Oracle: `/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v250-gateway-embedding/RUN/GA/dynamic_topology/evidence/tracking_aligned_v250/causal_gateway_embedding/m24_seed1301_h3_oracle/CAUSAL_GATEWAY_EMBEDDING_V250_H3_ORACLE.mat`
- Cache: `/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v250-gateway-embedding/RUN/GA/dynamic_topology/evidence/tracking_aligned_v250/causal_gateway_embedding/reference_cache/V250_V242_REFERENCE_M24_FORMATION_FOV_TEMPORAL_COUPLED_FORMATION_BRAID_SEED1301.mat`

Detailed distributions are retained in `receiver_stages.csv`, `label_stages.csv`, and the MAT record (including all active weights and per-source input existences).
