# Network-budget reallocated multi-source H=7 four-arm return screen

- Contract: `network-budget-reallocated-multi-source-h7-opened-return-screen-v95-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov-braided-handover / 41 / 112`
- Return times: `[112 113 114 115 116 117 118]`
- Intervention: `explicit frozen H=7 sequence`
- Generation commit: `50563314ef67322f17571a1776b15af9609402fa`
- Cache SHA-256: `d15783be8651b0bcaca0996cdc394ea4fa4c457cfca6e7bdb042c68dc63f15d4`
- Bank construction: `1.71 s`
- Proxy positive / realized positive: `3 / 3`
- Proxy TP / FP / FN: `3 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+0.425%`
- Best tail-safe mean gain: `+0.252%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree` | 0 | 1 | 0.00 | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `donor-only-ablation -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree` | 4+2+3 | 2 | 0.00 | -0.059651 | 1 | +0.023% | -0.000% | -0.000% | +0.070% | +0.260% | +0.869% | 1 |
| `sender-budget-reallocated-multi-source -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree` | 4+2+3 | 3 | 0.05 | +0.887889 | 1 | +0.252% | +0.305% | +0.000% | +0.514% | +1.331% | +0.754% | 1 |
| `sender-budget-reallocated-multi-source -> sender-budget-reallocated-multi-source -> sender-budget-reallocated-multi-source -> sender-budget-reallocated-multi-source -> sender-budget-reallocated-multi-source -> sender-budget-reallocated-multi-source -> sender-budget-reallocated-multi-source` | 4+2+3 | 3 | 0.05 | +0.887889 | 1 | +0.425% | -0.324% | -0.108% | +0.416% | +2.183% | +1.684% | 1 |

## Evidence boundary

V95 uses one already-opened braided-handover state per scale. A residual 0.05 message token is moved from its registered donor receiver to a physically reachable target receiver while keeping the sender unchanged. The dominant 0.70 inputs never move. The deployable candidate must preserve every sender message count and the network message budget exactly. It is compared with the same current physical-tree static route, a donor-only causal ablation, and the same selected route held fixed for the full horizon. Selection reads only current LMB posteriors, current association support, current physical links, link probabilities and past topology. Truth, future measurements, realized future links and tracking outcomes are unavailable until all four arms are frozen. These anchors are development evidence only.
