# V36 multi-state M24 source-only preflight

- Contract / generation commit: `formation-staggered-recovery-m24-v36-preflight-v1 / 3fc4a0e280dea00bfc934e5a0fb8989f6887acb0`
- Tracked dirty / untracked source: `0 / 0`
- Preset / seed / anchor times: `m24-formation-fov / 211 / [60 104 124]`
- Tracking scored: `0`
- Authorized states: `3 / 3`

## t=60

- Cache SHA-256: `05e3d62a50f3ca4dfa1d37c0b1d1c7e21cb1762e1965cfd6d87f9b98ca984bd5`
- Initial action / formations: `11 / [2 4]`
- Initial control / runtime replay: `33.15 / 137.94 s`
- Tracking authorized: `1`

| Time | Debt fractions | Selected | Released | Reserve |
|--:|:--|:--|:--|:--:|
| 60 | `[0.014165 0.029679 0.00032084 0.025586]` | `[2 4]` | `[]` | 1 |
| 61 | `[0.012581 0.024833 0.0086107 0.028125]` | `[2 4]` | `[]` | 1 |
| 62 | `[0.0081878 NaN 0.0070386 NaN]` | `[]` | `[]` | 1 |

## t=104

- Cache SHA-256: `83e07c28cdaeed614f9eaecb4d76d5307a4489903f19acc353acc90d0bfc3def`
- Initial action / formations: `12 / [1 2 4]`
- Initial control / runtime replay: `33.34 / 150.18 s`
- Tracking authorized: `1`

| Time | Debt fractions | Selected | Released | Reserve |
|--:|:--|:--|:--|:--:|
| 104 | `[0.093111 0.093179 0.018279 0.049808]` | `[1 2 4]` | `[]` | 1 |
| 105 | `[0.096523 0.091999 0.01715 0.046574]` | `[1 2]` | `4` | 1 |
| 106 | `[NaN NaN 0.017501 0.047871]` | `4` | `[]` | 1 |

## t=124

- Cache SHA-256: `fa9676ce8c5940760f1b0e837cab3a2057f96cb3c8a3f15b69fa1951caf30d05`
- Initial action / formations: `15 / [2 3 4]`
- Initial control / runtime replay: `32.74 / 144.45 s`
- Tracking authorized: `1`

| Time | Debt fractions | Selected | Released | Reserve |
|--:|:--|:--|:--|:--:|
| 124 | `[0.006876 0.10656 0.084208 0.024015]` | `[2 3 4]` | `[]` | 1 |
| 125 | `[0.0078084 0.098953 0.082402 0.024448]` | `[2 3]` | `4` | 1 |
| 126 | `[0.008015 NaN NaN 0.023919]` | `4` | `[]` | 1 |

## Decision

All three already-opened M24 states pass the full source-only runtime gate. One paired H=3 outcome per state is authorized under the unchanged v35 numerical gates.

## Evidence boundary

v36 reuses three already-opened M24 seed-211 posterior caches at t=60, 104, and 124. Each state first constructs the unchanged v35 controller and executes a full truth-free H=3 source rollout. Tracking is authorized state by state only after the clean rollout passes exact initial execution, posterior-content use, link-state use, label safety, and topology reserve. Offline outcomes retain the unchanged v35 gates. GNN, X36, X48, reserved seeds, and validation remain sealed until the replication gate passes.
