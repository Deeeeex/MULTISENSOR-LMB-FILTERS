# V74 receiver direct task-risk anchors

## m24-formation-fov-merge-split / t=80

| Route formations | Existence net / network | Affected Bayes objective | Minimum Bayes objective | Mean disagreement gain | Tail disagreement gain | Direct gate |
|:--|--:|:--|--:|--:|--:|:--:|
| `[]` | +0.000% | `[]` | +0.000000 | +0.000% | +0.000% | 1 |
| `3` | +0.638% | `0.0322772` | +0.032277 | -0.619% | -0.027% | 0 |

- Direct-gate candidate sets: `none`
- Position cutoff: `150.000`

## x36-formation-fov-merge-split / t=52

| Route formations | Existence net / network | Affected Bayes objective | Minimum Bayes objective | Mean disagreement gain | Tail disagreement gain | Direct gate |
|:--|--:|:--|--:|--:|--:|:--:|
| `[]` | +0.000% | `[]` | +0.000000 | +0.000% | +0.000% | 1 |
| `4` | +0.274% | `0.0992041` | +0.099204 | -0.407% | -0.596% | 0 |
| `5` | +0.747% | `0.0319276` | +0.031928 | -0.710% | -0.574% | 0 |
| `[4 5]` | +1.021% | `[0.0992041 0.0319276]` | +0.031928 | -1.103% | -1.107% | 0 |

- Direct-gate candidate sets: `none`
- Position cutoff: `150.000`

## Gate summary

- Any non-reference direct-gate candidate: `0`
- Receiver mode: `mixture-aware-heavy`
- Temporal propagation model used: `0`
- Tracking outcome read: `0`
- Route executed: `0`

## Evidence boundary

V74 evaluates the reference plus every feasible V73 formation subset at the two opened source anchors. It uses exact one-round mixture-aware receiver outcome distributions under current independent link uncertainty, then computes truth-free posterior Bayes-risk and mean/tail network disagreement. No route is executed and no tracking outcome, future measurement, truth, or temporal propagation model is read.
