# V278 missing-packet self-weight ablation

Source `eb7bc57f541af00f37636b61a1d2edaa919c51e6`; reference `c9c589d72209fcc8903b338fe77fd9fc26001409`; X36 seed1301, 160 steps.

| Arm | E-OSPA | Conditional RMSE | Focus consistency | Cardinality error | Attempted bytes |
|:--|--:|--:|--:|--:|--:|
| Fixed tree | 132.680 | 36.925 | 139.407 | 18.456 | 76871008 |
| V242 packet renormalization | 132.192 | 19.329 | 140.489 | 18.597 | 60090416 |
| V278 packet self-weight fallback | 131.961 | 20.263 | 139.273 | 18.518 | 60316640 |

| Comparison | E gain | RMSE gain | Consistency gain | Byte saving | Cardinality error delta | Min formation E / RMSE gain |
|:--|--:|--:|--:|--:|--:|:--|
| V278 over V242 | +0.175% | -4.836% | +0.865% | -0.376% | -0.079514 | -0.664% / -37.361% |
| V278 over fixed | +0.542% | +45.124% | +0.096% | +21.535% | +0.062153 | -1.155% / -4.079% |

Promising for the paired M24 follow-up: `0`.

Only the missing-neighbor weight rule changes; measurements, filter RNG, planned topology, directed delivery uniforms and powered-GM settings are shared. The rule applies to unavailable or empty neighbor inputs; label-level absence handling stays unchanged. It costs no additional messages or metadata, but recursive posterior sizes can change attempted payload bytes. This is a single-seed development ablation, not multiseed or cross-scale validation.

## Post-hoc common-cell RMSE readout

The normal process exit was observed after the result was saved. Measured
filter runtime is 46.0 minutes; this is not the wall time spent observing the
background session. No M24 follow-up was launched.

Restricting both arms to their common finite-RMSE sensor-time cells gives
V278/V242 means of 19.990875/18.953983 m, a 5.470572% deterioration. Finite
cells number 5625/5760 versus 5614/5760 before taking the intersection.
Thus the conditional-RMSE tradeoff persists under the same-cell comparison;
this still does not guarantee the same matched target identities.
The largest formation-relative RMSE loss is F3: 10.6439 to 14.6205 m.
These readouts do not change the frozen raw-metric follow-up gate.

Reproduce without rerunning filtering:

```matlab
s=load('RUN/GA/dynamic_topology/evidence/tracking_aligned_v278/x36_missing_packet_self_seed1301/MISSING_PACKET_SELF_WEIGHT_V278.mat');
a=s.result.candidate; b=s.result.reference;
x=a.positionRmseBySensorTime; y=b.positionRmseBySensorTime;
m=isfinite(x)&isfinite(y);
[mean(x(m)),mean(y(m)),100*(mean(y(m))-mean(x(m)))/mean(y(m))]
[b.perFormationPositionRmse(:),a.perFormationPositionRmse(:)]
```

Representative first output: `19.990875  18.953983  -5.470572`.
