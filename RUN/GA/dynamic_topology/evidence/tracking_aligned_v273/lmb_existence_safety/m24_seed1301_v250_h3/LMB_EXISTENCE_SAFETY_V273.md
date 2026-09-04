# V273 operator-aligned LMB existence-safety diagnostic

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`
- Analysis commit: `aa45855819702621e63ee5d3fd312647930d02a3`
- V250 oracle commit: `3248df0edeba8e8e24d897821a4922522cdec5a8`
- Anchors: `[70 84 151]`
- Stable safety signal found: `0`
- New tracking / GNN authorized: `0 / 0`
- Next decision: `close-contraction-first-gateway-substitution-family`
- Runtime: `81.7 s`

## Signal gate

| Risk signal | Card. rho by anchor | E-harm rho by anchor | LOAO balanced / recall / specificity | Useful retained | Pass |
|:--|:--|:--|:--|:--|:--:|
| `reference_weighted_retention_risk` | `[-0.22 0.496 0.346]` | `[-0.216 0.448 0.457]` | 0.479 / 0.125 / 0.833 | `[0 2 1]` | 0 |
| `maximum_receiver_retention_risk` | `[-0.227 0.554 0.352]` | `[-0.243 0.519 0.472]` | 0.479 / 0.125 / 0.833 | `[0 2 1]` | 0 |
| `maximum_formation_retention_risk` | `[-0.22 0.496 0.346]` | `[-0.216 0.448 0.457]` | 0.479 / 0.125 / 0.833 | `[0 2 1]` | 0 |
| `receiver_cardinality_loss` | `[-0.248 0.621 0.61]` | `[-0.202 0.592 0.683]` | 0.479 / 0.125 / 0.833 | `[0 2 1]` | 0 |
| `formation_mean_cardinality_loss` | `[-0.445 0.649 0.5]` | `[-0.333 0.541 0.529]` | 0.583 / 0.333 / 0.833 | `[2 1 1]` | 0 |
| `supported_label_retention_loss` | `[-0.161 0.561 0.484]` | `[-0.154 0.512 0.644]` | 0.507 / 0.125 / 0.889 | `[0 2 1]` | 0 |
| `map_threshold_crossing_count` | `[-0.591 0.325 0.385]` | `[-0.526 0.245 0.412]` | 0.410 / 0.042 / 0.778 | `[0 2 1]` | 0 |
| `map_threshold_crossing_mass` | `[-0.535 0.398 0.417]` | `[-0.497 0.293 0.457]` | 0.465 / 0.042 / 0.889 | `[0 2 1]` | 0 |
| `maximum_reference_weighted_shortfall` | `[-0.252 0.556 0.425]` | `[-0.265 0.56 0.605]` | 0.479 / 0.125 / 0.833 | `[0 2 1]` | 0 |
| `mean_reference_weighted_shortfall` | `[-0.226 0.447 0.454]` | `[-0.229 0.408 0.62]` | 0.479 / 0.125 / 0.833 | `[0 2 1]` | 0 |
| `maximum_normalized_existence_drift` | `[-0.208 0.0348 0.0425]` | `[-0.285 -0.0617 0.171]` | 0.493 / 0.042 / 0.944 | `[1 2 1]` | 0 |
| `mean_normalized_existence_drift` | `[-0.188 0.255 0.0299]` | `[-0.246 0.2 0.135]` | 0.528 / 0.167 / 0.889 | `[0 2 1]` | 0 |
| `one_round_disagreement_increase_fraction` | `[0.403 -0.515 0.203]` | `[0.335 -0.447 0.174]` | 0.562 / 0.292 / 0.833 | `[1 2 1]` | 0 |

## Candidate evidence

| t | Candidate | Type | Retention risk | Min retention | MAP drops | One-round disagreement change | H=3 E gain | Min-formation E gain | Card-error delta | Harm | Useful |
|--:|--:|:--|--:|--:|--:|--:|--:|--:|--:|:--:|:--:|
| 70 | 1 | `v242-reference` | 0.000000 | 1.000000 | 0 | +0.000% | +0.0000% | +0.0000% | +0.00000 | 0 | 0 |
| 70 | 2 | `global-rank-profile` | 0.074881 | 0.000000 | 3 | -0.095% | +0.1081% | +0.0003% | -0.02778 | 0 | 0 |
| 70 | 3 | `global-rank-profile` | 0.036921 | 0.000000 | 1 | -0.007% | -0.0594% | -0.2579% | +0.01389 | 1 | 0 |
| 70 | 4 | `receiver-local-assignment` | 0.074881 | 0.000000 | 2 | +0.058% | +0.0624% | -0.0001% | -0.01389 | 0 | 1 |
| 70 | 5 | `receiver-local-assignment` | 0.011904 | 0.810427 | 1 | -0.218% | +0.0453% | +0.0000% | -0.01389 | 0 | 0 |
| 70 | 6 | `receiver-local-assignment` | 0.000000 | 1.000000 | 0 | -0.005% | -0.0669% | -0.2575% | +0.01389 | 1 | 0 |
| 70 | 7 | `receiver-local-assignment` | 0.000692 | 0.986470 | 0 | +0.071% | +0.0003% | +0.0000% | +0.00000 | 0 | 0 |
| 70 | 8 | `receiver-local-assignment` | 0.002580 | 0.948118 | 0 | -0.043% | +0.0004% | +0.0000% | +0.00000 | 0 | 0 |
| 70 | 9 | `receiver-local-assignment` | 0.000574 | 0.956504 | 0 | +0.036% | +0.0008% | -0.0014% | +0.00000 | 0 | 0 |
| 70 | 10 | `receiver-local-assignment` | 0.022519 | 0.271952 | 0 | +0.008% | -0.0676% | -0.2578% | +0.01389 | 1 | 0 |
| 70 | 11 | `receiver-local-assignment` | 0.036921 | 0.000000 | 1 | -0.010% | +0.0057% | -0.0002% | +0.00000 | 0 | 1 |
| 70 | 12 | `receiver-local-assignment` | 0.002234 | 0.940725 | 0 | -0.053% | +0.0003% | -0.0000% | +0.00000 | 0 | 0 |
| 70 | 13 | `receiver-local-assignment` | 0.009391 | 0.840704 | 1 | -0.201% | +0.0453% | +0.0000% | -0.01389 | 0 | 0 |
| 70 | 14 | `receiver-local-assignment` | 0.002636 | 0.906600 | 0 | +0.030% | -0.0678% | -0.2609% | +0.01389 | 1 | 0 |
| 70 | 15 | `receiver-local-assignment` | 0.041059 | 0.000000 | 1 | -0.083% | -0.0081% | -0.0318% | +0.00000 | 0 | 0 |
| 70 | 16 | `single-directed-arc` | 0.001701 | 0.928022 | 1 | +0.018% | -0.0001% | -0.0006% | +0.00000 | 0 | 0 |
| 70 | 17 | `single-directed-arc` | 0.001734 | 0.933018 | 1 | +0.017% | -0.0023% | -0.0098% | +0.00000 | 0 | 0 |
| 70 | 18 | `single-directed-arc` | 0.002188 | 0.945540 | 0 | -0.014% | -0.0011% | -0.0043% | +0.00000 | 0 | 0 |
| 70 | 19 | `single-directed-arc` | 0.000158 | 0.995271 | 0 | -0.001% | -0.0045% | -0.0193% | +0.00000 | 0 | 0 |
| 70 | 20 | `single-directed-arc` | 0.021193 | 0.272204 | 0 | +0.039% | -0.0677% | -0.2581% | +0.01389 | 1 | 0 |
| 70 | 21 | `single-directed-arc` | 0.001512 | 0.954524 | 0 | +0.001% | -0.0006% | -0.0024% | +0.00000 | 0 | 0 |
| 84 | 1 | `v242-reference` | 0.000000 | 1.000000 | 0 | +0.000% | +0.0000% | +0.0000% | +0.00000 | 0 | 0 |
| 84 | 2 | `global-rank-profile` | 0.021209 | 0.751063 | 1 | -0.398% | -0.2254% | -0.6512% | +0.04167 | 1 | 0 |
| 84 | 3 | `global-rank-profile` | 0.017421 | 0.633409 | 1 | -0.261% | -0.2507% | -0.5249% | +0.05556 | 1 | 0 |
| 84 | 4 | `receiver-local-assignment` | 0.018612 | 0.796206 | 0 | -0.279% | -0.0665% | -0.2635% | +0.01389 | 1 | 0 |
| 84 | 5 | `receiver-local-assignment` | 0.021209 | 0.751063 | 1 | -0.135% | -0.1576% | -0.6512% | +0.02778 | 1 | 0 |
| 84 | 6 | `receiver-local-assignment` | 0.000045 | 0.999666 | 0 | +0.006% | +0.0001% | -0.0001% | +0.00000 | 0 | 0 |
| 84 | 7 | `receiver-local-assignment` | 0.006152 | 0.870606 | 0 | +0.014% | -0.0013% | -0.0051% | +0.00000 | 0 | 0 |
| 84 | 8 | `receiver-local-assignment` | 0.007064 | 0.918137 | 0 | -0.040% | +0.0014% | +0.0000% | +0.00000 | 0 | 0 |
| 84 | 9 | `receiver-local-assignment` | 0.017421 | 0.734158 | 1 | -0.256% | -0.1271% | -0.5249% | +0.02778 | 1 | 0 |
| 84 | 10 | `receiver-local-assignment` | 0.012802 | 0.633409 | 0 | +0.008% | -0.1865% | -0.7148% | +0.04167 | 1 | 0 |
| 84 | 11 | `receiver-local-assignment` | 0.006733 | 0.853513 | 0 | +0.029% | -0.0018% | -0.0073% | +0.00000 | 0 | 0 |
| 84 | 12 | `receiver-local-assignment` | 0.004906 | 0.821667 | 0 | +0.103% | +0.2619% | -0.0000% | -0.05556 | 0 | 0 |
| 84 | 13 | `receiver-local-assignment` | 0.013260 | 0.807371 | 1 | -0.123% | +0.0668% | +0.0000% | -0.01389 | 0 | 0 |
| 84 | 14 | `receiver-local-assignment` | 0.004241 | 0.850468 | 0 | -0.017% | +0.0000% | -0.0000% | +0.00000 | 0 | 0 |
| 84 | 15 | `receiver-local-assignment` | 0.000235 | 0.995957 | 0 | +0.002% | -0.0001% | -0.0003% | +0.00000 | 0 | 0 |
| 84 | 16 | `single-directed-arc` | 0.004086 | 0.923646 | 1 | -0.039% | +0.0629% | +0.0000% | -0.01389 | 0 | 0 |
| 84 | 17 | `single-directed-arc` | 0.020151 | 0.701032 | 1 | -0.044% | +0.0124% | -0.0029% | +0.00000 | 0 | 1 |
| 84 | 18 | `single-directed-arc` | 0.008728 | 0.911967 | 0 | +0.038% | +0.1225% | +0.0000% | -0.02778 | 0 | 0 |
| 84 | 19 | `single-directed-arc` | 0.001496 | 0.917127 | 0 | +0.013% | +0.0020% | +0.0000% | +0.00000 | 0 | 0 |
| 84 | 20 | `single-directed-arc` | 0.006162 | 0.916049 | 0 | -0.155% | +0.1392% | +0.0000% | -0.02778 | 0 | 1 |
| 84 | 21 | `single-directed-arc` | 0.000000 | 1.000000 | 0 | +0.036% | +0.0026% | +0.0000% | +0.00000 | 0 | 0 |
| 151 | 1 | `v242-reference` | 0.000000 | 1.000000 | 0 | +0.000% | +0.0000% | +0.0000% | +0.00000 | 0 | 0 |
| 151 | 2 | `global-rank-profile` | 0.010503 | 0.781129 | 0 | +0.108% | -0.1175% | -0.7436% | +0.01389 | 1 | 0 |
| 151 | 3 | `global-rank-profile` | 0.018347 | 0.719349 | 1 | +0.002% | -0.4151% | -0.7498% | +0.06944 | 1 | 0 |
| 151 | 4 | `receiver-local-assignment` | 0.000687 | 0.999933 | 0 | -0.003% | -0.0000% | -0.0000% | +0.00000 | 0 | 0 |
| 151 | 5 | `receiver-local-assignment` | 0.005414 | 0.902969 | 0 | +0.034% | +0.0643% | +0.0000% | -0.01389 | 0 | 0 |
| 151 | 6 | `receiver-local-assignment` | 0.000624 | 0.969917 | 0 | -0.029% | -0.0039% | -0.0184% | +0.00000 | 0 | 0 |
| 151 | 7 | `receiver-local-assignment` | 0.010503 | 0.781129 | 0 | +0.106% | -0.1772% | -0.7436% | +0.02778 | 1 | 0 |
| 151 | 8 | `receiver-local-assignment` | 0.006950 | 0.990915 | 0 | -0.025% | +0.0003% | +0.0000% | +0.00000 | 0 | 0 |
| 151 | 9 | `receiver-local-assignment` | 0.000151 | 0.998206 | 0 | +0.005% | -0.1454% | -0.5741% | +0.02778 | 1 | 0 |
| 151 | 10 | `receiver-local-assignment` | 0.004516 | 0.930942 | 0 | -0.100% | -0.1728% | -0.8246% | +0.02778 | 1 | 0 |
| 151 | 11 | `receiver-local-assignment` | 0.018347 | 0.719349 | 1 | +0.121% | -0.1786% | -0.7494% | +0.02778 | 1 | 0 |
| 151 | 12 | `receiver-local-assignment` | 0.008096 | 0.972389 | 1 | +0.010% | +0.0000% | -0.0005% | +0.00000 | 0 | 0 |
| 151 | 13 | `receiver-local-assignment` | 0.001195 | 0.952620 | 0 | -0.210% | +0.2195% | +0.0000% | -0.04167 | 0 | 1 |
| 151 | 14 | `receiver-local-assignment` | 0.004713 | 0.913218 | 0 | -0.050% | -0.1685% | -0.8039% | +0.02778 | 1 | 0 |
| 151 | 15 | `receiver-local-assignment` | 0.004569 | 0.908290 | 0 | -0.104% | -0.0170% | -0.0728% | +0.00000 | 1 | 0 |
| 151 | 16 | `single-directed-arc` | 0.018620 | 0.778177 | 0 | -0.158% | -0.0263% | -0.6449% | +0.00000 | 1 | 0 |
| 151 | 17 | `single-directed-arc` | 0.011175 | 0.755426 | 1 | -0.096% | -0.3089% | -1.2932% | +0.05556 | 1 | 0 |
| 151 | 18 | `single-directed-arc` | 0.007382 | 0.781339 | 0 | +0.077% | -0.1776% | -0.5958% | +0.04167 | 1 | 0 |
| 151 | 19 | `single-directed-arc` | 0.004548 | 0.905090 | 0 | -0.030% | -0.1523% | -0.6376% | +0.02778 | 1 | 0 |
| 151 | 20 | `single-directed-arc` | 0.000028 | 0.998723 | 0 | -0.019% | -0.0021% | -0.0072% | +0.00000 | 0 | 0 |
| 151 | 21 | `single-directed-arc` | 0.003494 | 0.969313 | 0 | -0.042% | -0.1534% | -0.6422% | +0.02778 | 1 | 0 |

## Interpretation

No operator-aligned scalar existence-safety quantity passes the frozen cross-anchor gate. Contraction-first gateway substitution is therefore closed as a deployable method family on the current evidence; fitting a GNN to these three anchors is not authorized.

Conditional set RMSE is secondary: a lower RMSE is not called localization improvement when cardinality error or E-OSPA worsens. One-round disagreement is diagnostic only and is not a recursive consensus certificate.

## Evidence boundary

V273 reuses the completed V250 H=3 candidate outcomes from one M24 development seed. Predictor inputs contain current local LMB posteriors, current physical topology and link reliabilities, and past selected-topology history. The predictor executes the same mixture-aware label-wise fusion operator used by the tracker while enumerating current independent link deliveries. It uses no truth or future measurement, but it requires centralized current posteriors and does not charge a deployable control synopsis. Truth appears only in the already opened H=3 outcome labels. This is post-hoc method diagnosis, not full-episode, X36, multistyle, deployment or paper evidence.
