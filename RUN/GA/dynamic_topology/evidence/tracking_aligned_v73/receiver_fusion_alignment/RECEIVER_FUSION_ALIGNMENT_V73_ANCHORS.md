# V73 receiver-fusion alignment anchors

| Scale | Time | V71 selection | Aligned nomination | Aligned selection | Slots old / aligned | Local net by formation | Route changed | Route |
|:--|--:|:--|:--|:--|:--|:--|:--:|:--:|
| m24-formation-fov-merge-split | 80 | `3` | `3` | `3` | `2 / 2` | `[0.55671 0.23624 1.9717 0.34333]` | 1 | constructed |
| x36-formation-fov-merge-split | 52 | `[4 5]` | `[4 5]` | `[4 5]` | `4 / 4` | `[0.2887 0.33377 0.69951 1.4509 3.9185 0.025637]` | 1 | constructed |

## Aligned selected receiver slots

### m24-formation-fov-merge-split / t=80

| Receiver | Incumbent | Candidate | Gain | Harm | Net | Up / down crossings |
|--:|--:|--:|--:|--:|--:|:--|
| 14 | 10 | 21 | 0.635% | 0.002% | 0.634% | `1 / 0` |
| 17 | 19 | 20 | 0.024% | 0.019% | 0.005% | `0 / 0` |

### x36-formation-fov-merge-split / t=52

| Receiver | Incumbent | Candidate | Gain | Harm | Net | Up / down crossings |
|--:|--:|--:|--:|--:|--:|:--|
| 20 | 16 | 34 | 0.250% | 0.106% | 0.145% | `1 / 0` |
| 23 | 25 | 36 | 0.144% | 0.014% | 0.130% | `1 / 0` |
| 26 | 22 | 35 | 0.728% | 0.015% | 0.713% | `4 / 0` |
| 29 | 31 | 17 | 0.051% | 0.017% | 0.034% | `0 / 0` |

## Gate summary

- Receiver mode: `mixture-aware-heavy`
- Receiver-first ordering at both anchors: `1`
- Heavy neighbor event type at both anchors: `1`
- Any route changed from V71: `1`
- Tracking outcome read: `0`
- Route executed: `0`
- Temporal propagation model used: `0`

## Evidence boundary

V73 re-evaluates exactly the two opened V72 source anchors without executing a route or reading tracking outcomes. Direct receiver counterfactuals use the formal mixture-aware reference config, put the receiver first, and mark selected neighbors as delivered heavy messages. Link reliability remains an external expected-value factor. The check isolates receiver semantics before any temporal propagation model is introduced.
