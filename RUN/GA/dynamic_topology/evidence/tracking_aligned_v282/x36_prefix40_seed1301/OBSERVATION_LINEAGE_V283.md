# V283 observation-opportunity lineage

x36-formation-fov-temporal-coupled-formation-braid, seed 1301, unchanged reference steps 1--40. No filter rerun or candidate evaluated.

An input is marked informed if a component-mean observation opportunity (pD>0, including missed detections) occurred locally or reached it through an actually used positive-weight input. Observable-absence censoring counts as evidence. Pruned states lose their flag. This traces an opportunity path, not a confirmed detection or evidence magnitude.

| Steps | Mean never-informed input weight | Mean no-current-opportunity weight | Never-informed share of negative log odds | Weak pools with a strong input | Of those: any never-informed input |
|:--|--:|--:|--:|--:|--:|
| 1--5 | 0.7780 | 0.8595 | 89.860% | 89 | 82 |
| 6--10 | 0.5216 | 0.8546 | 59.457% | 117 | 17 |
| 11--15 | 0.3083 | 0.8469 | 34.556% | 124 | 2 |
| 16--20 | 0.1644 | 0.8336 | 17.885% | 119 | 1 |
| 21--25 | 0.0936 | 0.8209 | 9.858% | 125 | 1 |
| 26--30 | 0.0567 | 0.8136 | 5.973% | 131 | 0 |
| 31--35 | 0.0271 | 0.8011 | 2.930% | 129 | 0 |
| 36--40 | 0.0083 | 0.7949 | 0.937% | 110 | 0 |

| Steps | All label pools | Weak before spatial overlap | Weak pools with no strong input | Fraction of weak pools |
|:--|--:|--:|--:|--:|
| 1--5 | 4320 | 3756 | 3667 | 97.630% |
| 6--10 | 4320 | 3688 | 3571 | 96.828% |
| 11--15 | 4320 | 3634 | 3510 | 96.588% |
| 16--20 | 4319 | 3557 | 3438 | 96.654% |
| 21--25 | 4310 | 3513 | 3388 | 96.442% |
| 26--30 | 4288 | 3417 | 3286 | 96.166% |
| 31--35 | 4283 | 3324 | 3195 | 96.119% |
| 36--40 | 4264 | 3227 | 3117 | 96.591% |

A weak pool means weighted input log odds below zero, before spatial overlap; a strong input means existence at least 0.9. These are descriptive levels, not the MAP-cardinality extraction rule or a tuning grid. A historical opportunity flag does not quantify retained information and cannot distinguish detection from missed detection. Low late-prefix never-informed mass does not support ongoing untouched-prior dilution as a complete explanation of the late weak pools. It does not exclude a lasting cold-start effect: an early prior-handling intervention could change the subsequent trajectory and has not been evaluated. Weak-history attenuation and delayed fresh measurements also remain possible. Neither geometry-only censoring nor a new fusion rule is validated here.
