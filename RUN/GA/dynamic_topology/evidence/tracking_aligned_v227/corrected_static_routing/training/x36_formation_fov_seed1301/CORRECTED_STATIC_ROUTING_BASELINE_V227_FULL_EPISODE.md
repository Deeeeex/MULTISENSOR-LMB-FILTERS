# V227 corrected static versus dynamic routing

- Scene / seed / split: `x36-formation-fov / 1301 / training`
- Source commit: `4451b2a06fdf8a9a26f56c029351f298b1cddbaf`
- Fixed static baseline included: `1`
- Validation claim allowed: `0`

Structural parity: `72` messages/round; static routes `1`; dynamic routes `10`; dynamic changes `9`; mean edge Jaccard `0.7613`.

| Arm | Full E-OSPA | Focus E-OSPA | Full RMSE | Focus RMSE | Focus consensus | Attempted bytes | Routes |
|:--|--:|--:|--:|--:|--:|--:|--:|
| Static | 116.276 | 99.672 | 43.372 | 12.784 | 85.687 | 182497728 | 1 |
| Dynamic | — | — | — | — | — | — | — |

## Evidence boundary

V227 is a paired corrected-code routing baseline. Both arms use mixture-aware support-renormalized LMB-KLA, always-heavy complete posterior exchange, exactly two directed inputs per receiver, the same fusion-weight multiset, measurements, delivery uniforms and filter seed. Both arms use the same physical-identity-stable formation-backbone construction. The dynamic arm rebuilds it from the current geometry and link reliabilities. The static arm applies it once to the registered all-time-physical graph at the first page and returns that identical route throughout the episode. This isolates routing adaptation; it does not by itself validate V226 payload transfer or authorize a generalization claim.
