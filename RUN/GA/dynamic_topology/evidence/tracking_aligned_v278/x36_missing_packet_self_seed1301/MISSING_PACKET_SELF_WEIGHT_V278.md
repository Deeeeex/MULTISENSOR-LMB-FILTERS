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
