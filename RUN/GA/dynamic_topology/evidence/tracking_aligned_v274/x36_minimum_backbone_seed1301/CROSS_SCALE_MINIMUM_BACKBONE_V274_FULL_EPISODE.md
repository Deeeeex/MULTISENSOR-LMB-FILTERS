# V274 X36 minimum-backbone baseline

- Scene / seed: `x36-formation-fov-temporal-coupled-formation-braid / 1301`
- Generation commit: `c9c589d72209fcc8903b338fe77fd9fc26001409`
- Direction / material / inherited paper gate: `0 / 0 / 0`
- Next decision: `dynamic-repair-scales-but-minimum-backbone-removes-useful-inputs`

| Arm | Full E-OSPA | Full RMSE | Focus consistency | Attempted bytes | Messages / step |
|:--|--:|--:|--:|--:|:--:|
| Fixed formation tree | 132.680 | 36.925 | 139.407 | 76871008 | 66--72 |
| Full causal repair | 131.795 | 19.586 | 139.004 | 81258696 | 72--72 |
| V242 minimum causal backbone | 132.192 | 19.329 | 140.489 | 60090416 | 46--46 |

| Comparison | E-OSPA | RMSE | Focus consistency | Attempted-byte saving | Weakest formation E / RMSE |
|:--|--:|--:|--:|--:|:--|
| Full causal over fixed | +0.667% | +46.958% | +0.289% | -5.708% | -1.079% / +8.389% |
| V242 over fixed | +0.368% | +47.655% | -0.776% | +21.830% | -0.818% / +4.444% |
| V242 over full causal | -0.301% | +1.313% | -1.068% | +26.050% | -2.246% / -27.753% |

## Evidence boundary

V274 is one paired complete-episode X36 development result. Fixed routing, full causal repair and the V242 minimum backbone use the same scene, measurements, truth, communication schedule, fusion implementation and filter RNG. The fixed arm is the no-dynamic-routing baseline. This result may establish cross-scale headroom on the executed case only; it is not multiseed, multistyle, validation or paper-level evidence.
