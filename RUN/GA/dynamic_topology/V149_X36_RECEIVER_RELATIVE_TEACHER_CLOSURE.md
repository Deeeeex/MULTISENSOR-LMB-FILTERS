# V149 X36 receiver-relative teacher closure

## Decision

V149 is closed as a negative, repository-only development route.  The X36
intervention reaches the local 5% target, but the full and mature windows do
not preserve that gain, the weakest sensor and formation regress materially,
and the six registered output cells are not repaired.  The already-failed M24
gate also makes a joint M24/X36 method claim impossible.

## Registered X36 result

| Metric | Result | Required interpretation |
|:--|--:|:--|
| Intervention E-OSPA gain | +5.122% | local headroom only |
| Full-window E-OSPA gain | +0.025% | immaterial |
| Mature-window E-OSPA gain | -0.636% | fail |
| Minimum sensor gain | -5.797% | fail |
| Minimum formation gain | -5.316% | fail |
| Rejoined relay match | 100.000% | pass |
| Attempted-byte delta | -0.242% | pass |
| Overall mechanism gate | false | fail |

## Six-cell diagnostic

The targeted X36 cells had been improved by V142.  V149 returns them toward
the earlier high-error state rather than preserving that repair.

| Sensor | Output page | V142 E-OSPA | V149 E-OSPA | V149 improvement |
|--:|--:|--:|--:|--:|
| 4 | 4 | 69.462149 | 75.681250 | -6.219101 |
| 5 | 4 | 69.362415 | 75.698713 | -6.336298 |
| 6 | 4 | 69.463145 | 75.779651 | -6.316506 |
| 3 | 5 | 69.328063 | 69.485126 | -0.157064 |
| 4 | 5 | 69.408634 | 69.519588 | -0.110954 |
| 5 | 5 | 69.365425 | 69.605647 | -0.240222 |

## Consequence

Receiver-relative W/R role multiplexing is not retained as the primary method
direction.  Its privileged teacher access, weak full-window effect and large
local regressions make it unsuitable as either the final communication method
or a clean learning target.  Subsequent work moves to explicit per-source,
per-label KLA participation with ordinary mixture-aware LMB-KLA replay and
final-output-aligned oracle scoring.
