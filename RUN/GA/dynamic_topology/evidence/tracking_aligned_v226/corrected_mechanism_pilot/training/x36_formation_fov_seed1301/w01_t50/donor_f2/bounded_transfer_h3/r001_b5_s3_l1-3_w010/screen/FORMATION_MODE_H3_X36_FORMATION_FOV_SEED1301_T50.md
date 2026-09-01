# Bounded dominant-edge transfer H=3 return screen

- Contract: `bounded-dominant-edge-transfer-h3-opened-return-screen-v226-v1`
- Missing-label receiver mode: `fov-aware-censored`
- Preset / seed / time: `x36-formation-fov / 1301 / 50`
- Return times: `[50 51 52]`
- Intervention duration: `1` step(s)
- Generation commit: `5fe8dd8478fc1f6e4b8ad7a57b8e3ad9d5102709`
- Cache SHA-256: `ad0b00ac1045f9b5397ffbd3bfe1112516693288bb1a79594eecf49c61b994e6`
- Bank construction: `11.99 s`
- Proxy positive / realized positive: `1 / 1`
- Proxy TP / FP / FN: `1 / 0 / 0`
- Proxy action agreement: `1.000`
- Best mean gain: `+0.008%`
- Best tail-safe mean gain: `+0.008%`

| Action | Form. | Mode | Trust | Proxy obj. | Proxy allow | Mean gain | RMSE gain | Worst gain | Min form. gain | Cons. gain | Byte saving | B3 |
|:--|--:|--:|--:|--:|:--:|--:|--:|--:|--:|--:|--:|:--:|
| `reference-full-payload` | 0 | 1 | NaN | +0.000000 | 1 | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | +0.000% | 1 |
| `v226-bounded-donor-f2-to-beneficiary-f5-replace-dominant-nonself-s3-l1-3` | 2 | 2 | NaN | -0.356512 | 1 | +0.008% | +2.057% | +0.000% | +0.000% | -0.153% | +0.050% | 1 |

## Evidence boundary

V226 is a frozen development mechanism screen for receiver-specific partial label-graph rewiring. Each receiver preserves self evidence and total label-wise KLA weight, evaluates every registered transfer fraction, and selects the largest fraction whose fused label remains inside the two-sided ordinary-reference log-odds envelope. If no fraction passes, the receiver uses ordinary fusion. The complete Bernoulli Gaussian-mixture payload is charged before projection, so a smaller transfer fraction never creates artificial communication savings. The source and label remain offline teacher choices; this screen cannot support deployment or generalization claims.
