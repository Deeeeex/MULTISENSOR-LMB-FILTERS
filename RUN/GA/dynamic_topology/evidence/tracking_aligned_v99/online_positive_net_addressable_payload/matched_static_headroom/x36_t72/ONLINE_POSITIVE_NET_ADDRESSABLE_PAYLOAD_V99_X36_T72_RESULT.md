# V99 online positive-net payload: X36 t72

- Matched baseline: `matched-static-fixed-ccw-full-payload`
- Receiver semantics: `fov-aware-censored`
- Pairing: same cached posterior, measurements, link uniforms, filter RNG, static carrier graph, fusion weights, horizon and communication constraints.

| Arm | Mean E-OSPA | Gain vs static | Gain vs fixed V97 |
|:--|--:|--:|--:|
| Static full payload | 85.970277 | -- | -- |
| Fixed V97 | 83.896827 | +2.412% | -- |
| Online V99 | 83.561598 | +2.802% | +0.400% |

- Initial set: `[1 2 4 5]`
- Online sets by time: `[1 2 4 5] -> [1 2 3 4 5] -> [1 2 3 4 5]`
- Worst-sensor gain: `+7.669%`
- Minimum-formation gain: `+0.000%`
- Window / terminal consensus gain: `+5.149% / +10.177%`
- Attempted-byte saving: `+6.550%`
- Rolling B3 passed: `1`
- X36-t72 gate passed: `0`
- Four-anchor run authorized: `0`

## Evidence boundary

V99 keeps the matched static carrier graph and fusion weights, but recomputes the threshold-free safe positive-net receiver-formation set after every local update. Each decision uses the current pre-fusion posterior, current geometry and past selected graphs only; no truth, future measurements or future outcomes enter the selector. The static, fixed-V97 and online-V99 arms share the same cached posterior, measurements, link uniforms, filter RNG, horizon and communication constraints. Opened anchors remain development evidence only.
