# Output-aligned label-effective oracle H=8 return screen

- Contract: `output-aligned-label-effective-oracle-h8-opened-return-screen-v150-v1`
- Missing-label receiver mode: `support-renormalized`
- Preset / seed / time: `m24-formation-fov / 211 / 104`
- Return times: `[104 105 106 107 108 109 110 111]`
- Intervention duration: `1` step(s)
- Generation commit: `952822baa89d9bf4bdab1f4cd7c23b65015b61af`
- Cache SHA-256: `91f20f13ab361f525321d83679e89cce89b8f6676fed3b4f4ab420f22ecc43a3`
- Bank construction: `4.49 s`
- Proxy positive / realized positive: `3 / 3`
- Proxy TP / FP / FN: `3 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+1.038%`
- Best tail-safe mean gain: `+0.000%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | Worst gain | Min form. gain | Window cons. | Terminal cons. | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `current-fused-impact-top-2` | 0 | 2 | NaN | +23.999819 | 1 | +0.125% | +0.616% | -0.000% | +0.328% | +0.001% | -0.368% | 1 |
| `current-fused-impact-top-4` | 0 | 3 | NaN | +47.999557 | 1 | +0.188% | +0.616% | -0.000% | +0.452% | +0.001% | -0.425% | 1 |
| `current-fused-impact-top-8` | 0 | 4 | NaN | +95.995694 | 1 | +1.038% | +1.265% | -0.000% | +1.825% | +4.518% | -0.749% | 1 |

## Evidence boundary

The action bank uses only the opened current posterior, current link probabilities, payload estimates, and topology history. A selected route may be held for the registered duration without recomputing it from future data. Truth and future measurements score H=8 offline teacher targets only. The source state and action design are already opened, so this screen may select a learning target but cannot support M24 or X36 validation claims.
