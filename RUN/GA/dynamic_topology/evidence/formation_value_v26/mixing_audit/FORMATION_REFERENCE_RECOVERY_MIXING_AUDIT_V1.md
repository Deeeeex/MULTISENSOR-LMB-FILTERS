# Formation reference-recovery mixing audit

- Contract / protocol: `formation-reference-recovery-mixing-audit-result-v1 / formation-reference-recovery-mixing-audit-v1`
- Generation commit: `b22a34c0b66d8a21f10582f9fcff160ab4740c0d`
- Source commit / SHA-256: `27b2c879c634a4e32705c53cf413caa7df9fb6d1 / 098c4f374b19b9160b316f091db75c15ccfb529c068daa4b1b14541db47c8a39`
- Preset / seed / time / horizon: `m24-formation-fov / 211 / 72 / 5`
- Maximum action row-sum error: `1.11e-16`
- Formation aggregate maximum action difference: `1.11e-16`
- Formation action invariant: `1`
- Sensor Dobrushin saturated through H=5: `1`
- Topology-only short-horizon safety rejected: `1`
- GNN training authorized: `0`

## Reference formation-level fusion matrix

```text
0.9917   0.0083        0        0
        0   0.9917   0.0083        0
        0        0   0.9917   0.0083
   0.0083        0        0   0.9917
```

- Cross-formation mass by receiver formation: `[0.0083333333 0.0083333333 0.0083333333 0.0083333333]`
- All 256 actions reproduce this matrix within `1.11e-16`; they change sensor/gateway allocation but not coarse formation-to-formation mixing mass.

## H=5 topology-only metrics versus realized consensus

| Arm | Sequence | Sensor tau | Centered spectral | Max row dispersion | Window consensus | Final-step consensus | First tau<1 |
|--:|:--|--:|--:|--:|--:|--:|--:|
| 1 | `[1 1 1 1 1]` | 1.000000 | 1.581765 | 0.560145 | +0.000% | +0.000% | 7 |
| 2 | `[9 4 62 1 1]` | 1.000000 | 1.599928 | 0.565939 | -5.038% | -11.147% | 7 |
| 3 | `[57 4 14 1 1]` | 1.000000 | 1.572840 | 0.562398 | -3.282% | -9.238% | 6 |
| 4 | `[57 4 2 1 1]` | 1.000000 | 1.573351 | 0.556211 | -3.810% | -6.589% | 6 |
| 5 | `[6 49 13 1 1]` | 1.000000 | 1.534495 | 0.538321 | -2.800% | -4.824% | 6 |

## Weight-only reference-recovery extrapolation

| Arm | tau@1 | tau@3 | tau@5 | tau@7 | tau@10 | tau@20 | tau@30 |
|--:|--:|--:|--:|--:|--:|--:|--:|
| 1 | 1.000000 | 1.000000 | 1.000000 | 0.999035 | 0.990134 | 0.899068 | 0.749531 |
| 2 | 1.000000 | 1.000000 | 1.000000 | 0.999341 | 0.991087 | 0.901486 | 0.752597 |
| 3 | 1.000000 | 1.000000 | 1.000000 | 0.999275 | 0.991173 | 0.901908 | 0.753182 |
| 4 | 1.000000 | 1.000000 | 1.000000 | 0.998775 | 0.989902 | 0.898512 | 0.748828 |
| 5 | 1.000000 | 1.000000 | 1.000000 | 0.999324 | 0.991938 | 0.904530 | 0.756697 |

## Diagnostic conclusion

Rolling-B3 is non-discriminative for this action family. The sensor-level Dobrushin coefficient remains exactly one throughout the realized H=5 window, while the formation-level fusion matrix is action invariant. Centered product norms also fail as sufficient safety tests: candidates `[3 4 5]` have no worse H=5 centered spectral norm than reference but still have negative final-step consensus; the corresponding row-dispersion counterexamples are `[4 5]`.

The next action representation must change coarse cross-formation mixing mass and must condition value/risk on posterior and innovation heterogeneity. A graph-only contraction proxy cannot authorize learning or deployment from this evidence.

## Evidence boundary

This deterministic posthoc audit reads the opened v25 H=5 screen after its outcomes are known. It may diagnose the current action representation and motivate a preregistered redesign. It cannot select a deployable policy, authorize GNN training, or support M24, X36, or final-seed performance claims.
