# V98 X36 t72 within-window risk migration

- Preset / seed / start: `x36-formation-fov / 211 / 72`
- Receiver semantics: `fov-aware-censored`
- Features: current pre-fusion posterior, current geometry and past selected graph only; no truth or future measurement is used.

| t | Addressable | Selected positive-net | Downward crossings | Total risk | Addressable risk | Net benefit |
|--:|:--|:--|:--|--:|--:|--:|
| 72 | [1 2 4 5] | [1 2 4 5] | [0 0 1 0 0 1] | 0.00907 | 0.00752 | 0.00643 |
| 73 | [1 2 3 4 5 6] | [1 2 3 4 5] | [0 0 0 0 0 0] | 0.01223 | 0.01223 | 0.01059 |
| 74 | [1 2 3 4 5] | [1 2 3 4 5] | [0 0 0 0 0 2] | 0.01925 | 0.01892 | 0.01820 |

- Selected set changed: `1`
- Newly addressable formations: `[3 6]`
- Online re-selection mechanism supported: `1`
- Validation claim allowed: `0`
