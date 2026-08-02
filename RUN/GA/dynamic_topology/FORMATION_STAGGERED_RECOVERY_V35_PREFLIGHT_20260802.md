# V35 debt-coverage staggered-recovery preflight

- Contract / generation commit: `formation-staggered-recovery-v35-preflight-v1 / 00a0634ca9fa84c5ab3d933501353dc1740d3c7c`
- Tracked dirty / untracked source: `0 / 0`
- Preset / seed / decision time: `m24-formation-fov / 211 / 73`
- Source cache SHA-256: `60dfbf2615181cde046af15f42bba37c415ea0034cb7ce53685b79042bfaf762`
- Incumbent / selected suspension: `[2 3 4] / [2 4]`
- Released formation(s): `3`
- Retained debt coverage: `89.1389%`
- Incumbent disagreement: `1.979620010`
- Selected disagreement: `1.964836276`
- Relative disagreement improvement: `+0.7468%`
- Tracking scored / authorized: `0 / 1`
- Replay / control construction: `112.16 / 39.39 s`

- Full runtime source replay: `151.37 s`

## Source-only candidate table

| Released | Retained | Debt coverage | Disagreement improvement | Retention risk | Min label retention | Drops | Safe | Eligible |
|:--|:--|--:|--:|--:|--:|--:|:--:|:--:|
| `3` | `[2 4]` | 89.139% | +0.7468% | 0.000855 | 0.967007 | 0 | 1 | 1 |

## Full causal runtime trace

| Time | Debt fractions | Requested | Selected | Released | Staggered release | One-step reserve |
|--:|:--|:--|:--|:--|:--:|:--:|
| 72 | `[0.0068703 0.057005 0.02344 0.046506]` | `[2 3 4]` | `[2 3 4]` | `[]` | 0 | 1 |
| 73 | `[0.010305 0.052298 0.01237 0.049228]` | `[2 3 4]` | `[2 4]` | `3` | 1 | 1 |
| 74 | `[0.0074851 NaN 0.020101 NaN]` | `3` | `3` | `[]` | 0 | 1 |

## Decision

The clean source-only gate selects `suspend-f2-f4`: formation 3 is restored at t=73 while formations 2 and 4 remain protected. One paired H=3 tracking rerun on the already-opened t=72 state is authorized.

## Evidence boundary

v35 reuses only the frozen v30 M24 seed-211 t=72 trajectory. At t=73 it ranks previously suspended formations by current truth-free retention debt, proposes nested low-debt releases while preserving at least 80 percent of the incumbent positive debt, and accepts only an exact label-safe, rolling-B3-safe route that improves one-round disagreement by at least 0.25 percent relative to continued suspension. Truth and future outcomes are sealed while a full source-only H=3 rollout verifies the causal [2,3,4] to [2,4] to [3] runtime trace. Only then may a clean preflight authorize one paired H=3 rerun. No additional M24 state, GNN, X36, X48, or validation is opened.
