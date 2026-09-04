# V272 temporally balanced minimum backbone

- Generation commit: `eaac7bc73c543dc3f965f4bec35c98e22082255f`
- Structural seed: `1301`
- Tracking screen authorized: `1`
- Next decision: `run-paired-m24-event-window-before-full-episode`

| Scene | Receiver coverage | Maximum receiver share | Eligible coverage / missed opportunities | Hmax mean row TV | Hmax minimum external influence | Link ratio / drop | Gate |
|:--|:--|:--|:--|:--|:--|:--|:--:|
| m24-formation-fov-temporal-coupled-formation-braid | 0.667 -> 1.000 | 0.618 -> 0.197 | 0.833 -> 1.000 / 160 -> 5 | 0.751 -> 0.741 (+1.34%) | 0.0597 -> 0.0625 (+4.55%) | 0.900 / 0.050 | 1 |
| x36-formation-fov-temporal-coupled-formation-braid | 0.667 -> 1.000 | 0.614 -> 0.201 | 0.861 -> 1.000 / 160 -> 5 | 0.827 -> 0.819 (+1.00%) | 0.0595 -> 0.0604 (+1.45%) | 0.900 / 0.050 | 1 |

## Gate components

| Scene | Hard | Coverage | Concentration | Opportunity | Mixing | Quality |
|:--|:--:|:--:|:--:|:--:|:--:|:--:|
| m24-formation-fov-temporal-coupled-formation-braid | 1 | 1 | 1 | 1 | 1 | 1 |
| x36-formation-fov-temporal-coupled-formation-braid | 1 | 1 | 1 | 1 | 1 | 1 |

## Communication and route behavior

| Scene / arm | Messages | Mean / minimum cross-link reliability | Receiver / sender coverage | Receiver / sender maximum share | Route-change fraction |
|:--|:--|:--|:--|:--|--:|
| m24-formation-fov-temporal-coupled-formation-braid / V242 | 30--30 | 0.843 / 0.270 | 0.667 / 0.667 | 0.618 / 0.618 | 0.132 |
| m24-formation-fov-temporal-coupled-formation-braid / V272 | 30--30 | 0.818 / 0.249 | 1.000 / 1.000 | 0.197 / 0.355 | 1.000 |
| x36-formation-fov-temporal-coupled-formation-braid / V242 | 46--46 | 0.848 / 0.270 | 0.667 / 0.667 | 0.614 / 0.614 | 0.233 |
| x36-formation-fov-temporal-coupled-formation-braid / V272 | 46--46 | 0.824 / 0.245 | 1.000 / 1.000 | 0.201 / 0.290 | 1.000 |

## Decision

The same-message-count V272 route clears the registered M24 and X36 structural gates. Run one paired M24 event-window tracking screen before any full-episode claim.

## Evidence boundary

V272 replays the V242 and temporally balanced routes on the same physical graph and link-reliability schedules. No filter state, measurement, truth, tracking outcome or realized delivery is read. The report can authorize one paired development screen, but cannot establish tracking benefit, deployment value or generalization.
