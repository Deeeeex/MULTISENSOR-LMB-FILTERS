# V125 outward reference carrier: X36 t72 H=8

- V113 baselines reused: `1`
- V123 candidate screen reused: `0`
- Gate passed: `0`

| Mean E-OSPA | Gain vs CW | vs V113 | Mature min | Min form. | Terminal form. | F6 peers | Worst | Bytes | Gate |
|--:|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| 78.350770 | +4.221% | +0.164% | +1.830% | +0.000% | +0.000% | +0.000% | +6.836% | +2.421% | 0 |

## Diagnostics

- Formation gains: `[0.3078 0.9785 10.62 3.754 9.913 0]%`
- Terminal formation gains: `[4.351 4.858 16.38 9.566 6.645 0]%`
- Per-page gains: `[0.9093 3.021 4.154 2.773 1.83 7.471 6.845 7.184]%`
- Window / terminal consensus: `+10.778% / +16.163%`
- Rolling B3: `1`

## Evidence boundary

V125 is a privileged paired X36 seed-211 t=72 H=8 causal upper bound. It preserves every V124 graph row, fusion weight and receiver-side payload decision. Only the posterior transmitted on the persistent sensor-27-to-sensor-32 F5-to-F6 edge is replaced, page by page, by the same sender posterior captured from the paired clockwise full-payload arm. F5 therefore keeps the V124 protected local state while F6 receives the counterfactual static-carrier state. This requires an alternative-arm shadow trajectory and is not deployable; it only tests whether downstream state propagation is the remaining performance bottleneck. Measurements, delivery uniforms, filter RNG, topology, weights and byte accounting remain paired. No validation or generalization claim is authorized.
