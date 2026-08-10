# Network-budget reallocated multi-source H=5 four-arm return screen

- Contract: `network-budget-reallocated-multi-source-h5-opened-return-screen-v95-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `m24-formation-fov-braided-handover / 41 / 104`
- Return times: `[104 105 106 107 108]`
- Intervention: `explicit frozen H=5 sequence`
- Generation commit: `50563314ef67322f17571a1776b15af9609402fa`
- Cache SHA-256: `fc56d2f3b54a7cd353dee0d608e11997d43b4f47809e792e63d865adcc7e9f89`
- Bank construction: `0.46 s`
- Proxy positive / realized positive: `3 / 1`
- Proxy TP / FP / FN: `1 / 2 / 0`
- Proxy action agreement: `0.333`
- Best mean gain: `+0.000%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree` | 0 | 1 | 0.00 | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `donor-only-ablation -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree` | 2+3 | 2 | 0.00 | -0.018732 | 1 | -0.000% | +0.000% | -0.000% | -0.001% | +0.000% | +0.846% | 1 |
| `sender-budget-reallocated-multi-source -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree -> matched-static-physical-tree` | 2+3 | 3 | 0.05 | +0.239438 | 1 | +0.000% | +0.000% | -0.000% | -0.199% | -0.000% | +0.000% | 1 |
| `sender-budget-reallocated-multi-source -> sender-budget-reallocated-multi-source -> sender-budget-reallocated-multi-source -> sender-budget-reallocated-multi-source -> sender-budget-reallocated-multi-source` | 2+3 | 3 | 0.05 | +0.239438 | 1 | -0.082% | +0.000% | -0.380% | -0.711% | -0.753% | +0.000% | 1 |

## Evidence boundary

V95 uses one already-opened braided-handover state per scale. A residual 0.05 message token is moved from its registered donor receiver to a physically reachable target receiver while keeping the sender unchanged. The dominant 0.70 inputs never move. The deployable candidate must preserve every sender message count and the network message budget exactly. It is compared with the same current physical-tree static route, a donor-only causal ablation, and the same selected route held fixed for the full horizon. Selection reads only current LMB posteriors, current association support, current physical links, link probabilities and past topology. Truth, future measurements, realized future links and tracking outcomes are unavailable until all four arms are frozen. These anchors are development evidence only.
