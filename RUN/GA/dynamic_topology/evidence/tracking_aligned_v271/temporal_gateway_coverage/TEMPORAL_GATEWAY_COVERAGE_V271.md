# V271 temporal gateway coverage diagnostic

- Generation commit: `44df1c06d9d7e7a2615503c263a0f2904dd348cb`
- Structural seed: `1301`
- V272 authorized: `1`
- Next decision: `implement-v272-temporally-balanced-minimum-backbone`

| Scene | N / F | Minimum receiver coverage | Maximum receiver share | Maximum mean / absolute age | Mean / maximum local distance | Hmax mean / maximum row TV | Concentrated / mixing headroom |
|:--|:--:|--:|--:|:--|:--|:--|:--:|
| m24-formation-fov-temporal-coupled-formation-braid | 24 / 4 | 0.667 | 0.618 | 44.116 / 160 | 2.406 / 5 | H12 0.751 / 0.996 | 1 / 1 |
| x36-formation-fov-temporal-coupled-formation-braid | 36 / 6 | 0.667 | 0.614 | 48.791 / 160 | 2.446 / 5 | H12 0.827 / 1.000 | 1 / 1 |

## Per-formation route concentration

| Scene | Formation | Receiver coverage | Maximum receiver share | Mean / maximum age | Mean / maximum local distance |
|:--|--:|--:|--:|:--|:--|
| m24-formation-fov-temporal-coupled-formation-braid | 1 | 0.667 | 0.378 | 44.011 / 160 | 1.647 / 5 |
| m24-formation-fov-temporal-coupled-formation-braid | 2 | 1.000 | 0.533 | 41.384 / 148 | 2.038 / 5 |
| m24-formation-fov-temporal-coupled-formation-braid | 3 | 0.667 | 0.458 | 41.397 / 160 | 1.231 / 5 |
| m24-formation-fov-temporal-coupled-formation-braid | 4 | 1.000 | 0.618 | 44.116 / 150 | 2.406 / 5 |
| x36-formation-fov-temporal-coupled-formation-braid | 1 | 0.667 | 0.410 | 42.876 / 160 | 1.519 / 5 |
| x36-formation-fov-temporal-coupled-formation-braid | 2 | 0.833 | 0.565 | 48.791 / 160 | 2.118 / 5 |
| x36-formation-fov-temporal-coupled-formation-braid | 3 | 0.833 | 0.452 | 30.276 / 160 | 1.052 / 4 |
| x36-formation-fov-temporal-coupled-formation-braid | 4 | 1.000 | 0.399 | 22.098 / 102 | 1.249 / 5 |
| x36-formation-fov-temporal-coupled-formation-braid | 5 | 0.833 | 0.500 | 34.493 / 160 | 1.271 / 5 |
| x36-formation-fov-temporal-coupled-formation-braid | 6 | 1.000 | 0.614 | 45.328 / 156 | 2.446 / 5 |

## Finite-horizon nominal mixing

| Scene | Horizon | Mean row TV | Maximum row TV |
|:--|--:|--:|--:|
| m24-formation-fov-temporal-coupled-formation-braid | 3 | 0.919755 | 1.000000 |
| m24-formation-fov-temporal-coupled-formation-braid | 6 | 0.849992 | 1.000000 |
| m24-formation-fov-temporal-coupled-formation-braid | 12 | 0.750579 | 0.995593 |
| x36-formation-fov-temporal-coupled-formation-braid | 3 | 0.946465 | 1.000000 |
| x36-formation-fov-temporal-coupled-formation-braid | 6 | 0.898109 | 1.000000 |
| x36-formation-fov-temporal-coupled-formation-braid | 12 | 0.826983 | 0.999951 |

## M24 tail alignment

- Formation / window: `F4 / t=58--73`
- Direct-input age / RMSE correlation: `0.726147`
- Local-distance / RMSE correlation: `0.078817`
- Event imbalance gate: `1`

| Sensor UID | Direct-input fraction | Mean direct age | Mean local distance | Mean RMSE |
|--:|--:|--:|--:|--:|
| 70040401 | 0.000 | 65.500 | 3.000 | 28.777793 |
| 70040402 | 0.000 | 65.500 | 4.000 | 26.180937 |
| 70040403 | 0.000 | 65.500 | 5.000 | 23.288584 |
| 70040404 | 1.000 | 0.000 | 0.000 | 19.393192 |
| 70040405 | 0.000 | 65.500 | 1.000 | 28.709835 |
| 70040406 | 0.000 | 65.500 | 2.000 | 33.463348 |

## Decision

V242 leaves a cross-scale temporal concentration and mixing gap that is aligned with the opened M24 tail. Implement V272 as a same-message-count temporal balancing policy before running tracking.

## Evidence boundary

V271 replays the causal V242 route without changing posterior state, measurements, fusion weights, messages or random streams. M24 tracking outcomes are joined only after route construction. The diagnostic may authorize one development action family; it is not a tracking, deployment, validation or generalization result.
