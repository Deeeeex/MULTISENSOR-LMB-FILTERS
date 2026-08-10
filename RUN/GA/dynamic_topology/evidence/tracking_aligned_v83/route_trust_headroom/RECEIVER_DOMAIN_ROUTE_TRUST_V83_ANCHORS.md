# V83 receiver-domain route-and-trust H=3 headroom

- Both scales pass the headroom gate: `0`
- Online selector available: `0`
- Tracking outcome read: `1`

## m24-formation-fov-merge-split / t=80

| Trust | Mean tracking | Worst sensor | Minimum formation | Window consensus | Terminal consensus | Attempted bytes | B3 | Gate |
|--:|--:|--:|--:|--:|--:|--:|:--:|:--:|
| 0.05 | +0.668% | +0.000% | +0.000% | -0.625% | -1.352% | -0.074% | 1 | 0 |
| 0.10 | -0.027% | +0.000% | -0.151% | -0.707% | -2.140% | -0.074% | 1 | 0 |
| 0.15 | -0.582% | +0.000% | -3.175% | -0.456% | -1.664% | +0.047% | 1 | 0 |
| 0.20 | -0.748% | +0.000% | -4.078% | -0.392% | -1.585% | +0.059% | 1 | 0 |

- Best action / trust: `receiver-domain-route-trust-05 -> reference -> receiver-domain-route-trust-05 / 0.05`
- Best mean gain: `+0.668%`
- Headroom gate passed: `0`

## x36-formation-fov-merge-split / t=52

| Trust | Mean tracking | Worst sensor | Minimum formation | Window consensus | Terminal consensus | Attempted bytes | B3 | Gate |
|--:|--:|--:|--:|--:|--:|--:|:--:|:--:|
| 0.05 | +0.201% | +0.000% | +0.000% | +0.295% | +0.167% | -0.006% | 1 | 0 |
| 0.10 | +0.118% | +0.000% | +0.000% | +0.253% | +0.218% | +0.075% | 1 | 0 |
| 0.15 | -0.034% | +0.000% | -0.219% | -0.012% | +0.221% | +0.075% | 1 | 0 |
| 0.20 | -0.183% | +0.000% | -1.173% | -0.156% | -0.015% | +0.075% | 1 | 0 |

- Best action / trust: `receiver-domain-route-trust-05 -> reference -> receiver-domain-route-trust-05 / 0.05`
- Best mean gain: `+0.201%`
- Headroom gate passed: `0`

## Evidence boundary

V83 is a paired H=3 action-space headroom screen on the two already opened merge-split anchors. It retains only the V75 direct-safe V71 receiver-domain sender replacements. Every candidate preserves the 0.70 within-formation dominant input, the physical route, and the exact directed-message count, but raises the selected cross-formation trust from 0.05 to one frozen point in {0.05,0.10,0.15,0.20}; additional trust is funded only from receiver self-weight. The candidate route is used in rounds one and three, with a current-physical-tree reference round between them so rolling-B3 remains valid. The complete sequence is frozen before truth and future measurements score paired H=3 tracking returns. This measures action-family headroom only and is not an online selector, model-training result, or validation claim.
