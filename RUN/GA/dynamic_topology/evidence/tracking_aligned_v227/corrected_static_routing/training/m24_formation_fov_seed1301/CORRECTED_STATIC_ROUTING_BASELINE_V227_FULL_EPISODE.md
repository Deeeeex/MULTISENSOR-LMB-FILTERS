# V227 corrected static versus dynamic routing

- Scene / seed / split: `m24-formation-fov / 1301 / training`
- Source commit: `4451b2a06fdf8a9a26f56c029351f298b1cddbaf`
- Fixed static baseline included: `1`
- Validation claim allowed: `0`

Structural parity: `48` messages/round; static routes `1`; dynamic routes `6`; dynamic changes `5`; mean edge Jaccard `0.8453`.

| Arm | Full E-OSPA | Focus E-OSPA | Full RMSE | Focus RMSE | Focus consensus | Attempted bytes | Routes |
|:--|--:|--:|--:|--:|--:|--:|--:|
| Static | 93.024 | 64.208 | 33.988 | 8.585 | 62.584 | 92954592 | 1 |
| Dynamic | 87.587 | 55.898 | 31.546 | 8.797 | 59.379 | 85887600 | 6 |

## Dynamic over static

| Metric | Gain |
|:--|--:|
| Full E-OSPA | `+5.845%` |
| Focus E-OSPA | `+12.942%` |
| Full RMSE | `+7.185%` |
| Focus RMSE | `-2.466%` |
| Worst-sensor E-OSPA | `+6.381%` |
| Worst-sensor RMSE | `+7.379%` |
| Focus consistency | `+5.122%` |
| Attempted-byte saving | `+7.603%` |
| Weakest formation E-OSPA | `-1.306%` |
| Weakest formation RMSE | `-16.185%` |

- Joint mean direction passed: `1`
- Nonnegative formation tail passed: `0`

## Evidence boundary

V227 is a paired corrected-code routing baseline. Both arms use mixture-aware support-renormalized LMB-KLA, always-heavy complete posterior exchange, exactly two directed inputs per receiver, the same fusion-weight multiset, measurements, delivery uniforms and filter seed. Both arms use the same physical-identity-stable formation-backbone construction. The dynamic arm rebuilds it from the current geometry and link reliabilities. The static arm applies it once to the registered all-time-physical graph at the first page and returns that identical route throughout the episode. This isolates routing adaptation; it does not by itself validate V226 payload transfer or authorize a generalization claim.
