# Formation H=3 duration probe

- Contract / protocol: `formation-h3-duration-probe-result-v1 / formation-h3-duration-probe-v1`
- Generation commit: `4c38bb6079936dbdb889952eff98861ffc8689a0`
- Cache protocol / generation commit: `formation-h3-event-conditioned-sentinel-v1 / c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53`
- Preset / seed / times: `m24-formation-fov / 211 / [60 72 104]`
- Candidate duration: `3` steps
- One-step strict gains: `[1.59066 0.024472 0.516904]`
- Duration strict gains: `[1.10478 0 0]`
- Duration minus one-step: `[-0.485878 -0.024472 -0.516904]` points

| Time | Action | One-step | Duration candidate targets | Duration strict |
|--:|:--|--:|:--|:--|
| 60 | formations-1-2-dynamic-trust-0.30 | +1.591% | `[1.10478 0 0.104182 3.76242 0.999954 1.06002]` | formations-1-2-dynamic-trust-0.30 (+1.105%) |
| 72 | formation-2-dynamic-trust-0.70 | +0.024% | `[0.105499 0 0.100596 -0.198963 1.03373 0.199926]` | reference (+0.000%) |
| 104 | formation-4-dynamic-trust-0.50 | +0.517% | `[0.629678 0 0 1.27943 -0.151744 -0.15604]` | reference (+0.000%) |

## Evidence boundary

The v16 actions are the outcome-inspected one-step strict oracles from seed 211 and therefore privileged. Each candidate holds its already-constructed adjacency and fusion weights for all three H=3 steps without reading future data; the reference arm continues to recompute the registered fixed reference after step one. This probe may test duration headroom only. Seeds 223, 227, X36, and all final seeds remain unopened.
