# V276 zipper-merge scene preflight

- Generation commit: `fbbe3d3518b96f5fe0ec2394bb61091fbe5c2bdf`
- Geometry seed: `1301`
- Geometry gate passed: `1`
- Cross-scale timing matched: `1`
- Tracking outcome authorized: `0`
- Next decision: `freeze-scene-before-any-tracking-and-use-after-method-selection`

| Scene | Valid | Physical connected | Fixed-tree feasible | Failure episode | Alternative edges | Planned handoffs | Actual owner changes in failure |
|:--|:--:|:--:|--:|:--:|--:|:--:|--:|
| m24-formation-fov-zipper-merge | 1 | 1 | 0.662 | 54--107 | 1 | [57 104] | 0.690 |
| x36-formation-fov-zipper-merge | 1 | 1 | 0.656 | 53--107 | 2 | [57 104] | 0.681 |

## Geometry and observation envelope

| Scene | Nodes / targets | Min sensor-target distance | Sensor speed / accel | Target speed / accel | Blackout / focus blackout | Focus handovers |
|:--|:--:|--:|:--|:--|:--|--:|
| m24-formation-fov-zipper-merge | 24 / 16 | 55.091 | 7.228 / 0.255 | 9.110 / 0.494 | 0.075 / 0.059 | 16 |
| x36-formation-fov-zipper-merge | 36 / 24 | 50.399 | 7.804 / 0.327 | 9.564 / 0.540 | 0.049 / 0.055 | 26 |

## Role in the scenario suite

Zipper merge is the non-radial positive stress case: two parallel platoons enter one shared bottleneck, change longitudinal neighbours, and split again. The physical formation graph stays connected while the initial tree fails during the same interval as the registered target handoffs. Parallel convoy and linear relay remain useful negative controls because their fixed routes stay available throughout; formation braid remains the development case.

## Evidence boundary

V276 uses generated sensor/target geometry and physical reachability only. It does not run the LMB filter and reads no posterior, realized delivery, tracking outcome or method score. Passing this gate qualifies a non-radial scene for a later paired experiment; it does not establish method benefit or generalization.
