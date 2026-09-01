# V235 corrected dynamic-routing tail localization

- Pair: `m24-formation-fov / seed 1301`
- Dynamic route changes: `5` at `42,57,68,70,153`
- Worst formation: `F4`
- Worst formation full E-OSPA gain: `-1.306%`
- Worst formation full RMSE gain: `-16.185%`

| Formation | Full E-OSPA gain | Full RMSE gain | Negative E pages | Negative R pages | Both-worse pages |
|--:|--:|--:|--:|--:|--:|
| F1 | `+5.421%` | `+6.475%` | 36 | 63 | 26 |
| F2 | `+16.547%` | `+7.907%` | 6 | 42 | 2 |
| F3 | `+2.383%` | `+33.601%` | 46 | 50 | 32 |
| F4 | `-1.306%` | `-16.185%` | 63 | 90 | 43 |

## Worst-formation route segments

| Segment | Times | Changed for F4 | Differs from static | Dynamic cross source->receiver | Static cross source->receiver | E gain | R gain | Both-worse fraction |
|--:|:--|:--:|:--:|:--|:--|--:|--:|--:|
| 1 | 1-41 | `0` | `1` | `S18->S20, S2->S24` | `S2->S19, S13->S20` | `+0.025%` | `+3.276%` | 4.9% |
| 2 | 42-56 | `0` | `1` | `S18->S20, S2->S24` | `S2->S19, S13->S20` | `+15.235%` | `+4.276%` | 0.0% |
| 3 | 57-67 | `1` | `1` | `S13->S20, S2->S24` | `S2->S19, S13->S20` | `-0.842%` | `-6.119%` | 18.2% |
| 4 | 68-69 | `0` | `1` | `S13->S20, S2->S24` | `S2->S19, S13->S20` | `-72.429%` | `+0.660%` | 50.0% |
| 5 | 70-152 | `1` | `0` | `S2->S19, S13->S20` | `S2->S19, S13->S20` | `-5.737%` | `-27.084%` | 38.6% |
| 6 | 153-160 | `0` | `0` | `S2->S19, S13->S20` | `S2->S19, S13->S20` | `-6.896%` | `-13.251%` | 75.0% |

Both-worse runs for F4: `35-36 (2), 63-63 (1), 65-65 (1), 68-68 (1), 70-82 (13), 88-98 (11), 102-104 (3), 110-111 (2), 138-138 (1), 150-151 (2), 155-160 (6)`.

## Evidence boundary

V235 is an offline localization of the completed V227 M24 pair. The reconstructed routes use only current geometry and link probability, but the tail labels use paired tracking outcomes. It may guide a causal formation-level fallback design; it is not an online policy, held-out result or tracking guarantee.
