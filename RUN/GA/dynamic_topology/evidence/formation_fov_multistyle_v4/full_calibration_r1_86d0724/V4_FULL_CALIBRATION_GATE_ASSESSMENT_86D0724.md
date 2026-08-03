# Multistyle v4 development geometry gate assessment

## Frozen evidence boundary

- Executable scene source: `86d0724`.
- Open development seeds: `[41, 43, 47, 53, 59]`.
- Presets: M24/X36 convoy and M24/X36 relay.
- All `20/20` realizations passed structural, kinematic, topology, outage,
  target-activity and cross-set safety validation.
- No tracking filter was run.  This evidence establishes scene geometry
  only; it does not establish a routing or estimation gain.

## Development decision

| Style | Decision | Main evidence |
|:--|:--|:--|
| Offset-corridor convoy | **GO for a separately frozen held-out geometry gate** | The v3 planar near-collision is removed; M24/X36 retain handovers and overlap with at least `35.98 m` sensor-target separation. |
| Linear relay | **GO for a separately frozen held-out geometry gate** | The v3 lane repair is unchanged and again passes target-level observability and safety checks. |
| Orthogonal crossing | **Stress-only** | v4 did not alter or reopen the rejected crossing main-scene geometry. |

Tracking remains unauthorized.  Development success permits only a later
source version that freezes the gates and unopened seed manifest below.

## Convoy result

| Metric across five seeds | M24 | X36 |
|:--|--:|--:|
| Global blackout | `0.003--0.007` | `0.004--0.005` |
| Focus blackout | `0` | `0` |
| Worst-target blackout | `0.013--0.025` | `0.019--0.025` |
| Longest blackout | `2--4` steps | `3--4` steps |
| Single-formation visibility | `0.526--0.548` | `0.410--0.446` |
| Multi-formation visibility | `0.445--0.467` | `0.551--0.585` |
| Focus target load / sensor-time | `4.54--4.58` | `4.72--4.76` |
| Focus handovers | `21--23` | `34--38` |
| Ownership entropy | `0.958--0.968` | `0.982--0.986` |
| Target-target separation | `20.00 m` | `20.00 m` |
| Sensor-target separation | `37.64--44.02 m` | `35.98--40.50 m` |

For the same seed index, X36/M24 focus-load ratio is approximately
`1.037--1.043`.  The larger scene adds a locally identical lane pair and
increases multi-formation support and normalized handovers without adding a
blackout penalty.  Numeric seed reuse is a matched perturbation index, not a
claim that the two random streams are statistically identical after X36
draws its additional formation jitters.

## Relay result

| Metric across five seeds | M24 | X36 |
|:--|--:|--:|
| Global blackout | `0.004--0.006` | `0.004--0.008` |
| Focus blackout | `0.001` | `0.004--0.006` |
| Worst-target blackout | `0.062--0.081` | `0.062--0.100` |
| Longest blackout | `10--12` steps | `5--9` steps |
| Single-formation visibility | `0.427--0.446` | `0.389--0.411` |
| Multi-formation visibility | `0.549--0.568` | `0.581--0.605` |
| Focus target load / sensor-time | `5.22--5.25` | `4.98--5.02` |
| Focus handovers | `40` | `88` |
| Ownership entropy | `0.995--0.996` | `0.989--0.990` |
| Target-target separation | `12.34 m` | `14.77 m` |
| Sensor-target separation | `36.31--43.95 m` | `33.43--36.64 m` |

The relay values match the source-frozen v3 audit.  The suite-version update
therefore did not silently change its geometry.

## Preregistered v5 absolute gates

Every seed and scale must pass every applicable row.  A favorable seed or
style cannot compensate for a failure.

| Per-realization metric | Convoy | Relay |
|:--|--:|--:|
| Global blackout | `<= 0.025` | `<= 0.010` |
| Focus blackout | `<= 0.010` | `<= 0.010` |
| Worst-target blackout | `<= 0.050` | `<= 0.120` |
| Longest blackout | `<= 8` steps | `<= 14` steps |
| Single-formation visibility | `>= 0.35` | `>= 0.35` |
| Multi-formation visibility | `>= 0.40` | `>= 0.50` |
| Focus target load / sensor-time | `4.0--5.25` | `4.5--5.5` |
| Normalized handovers | `handovers / targets >= 1.0` | `handovers / (targets * (formations-1)) >= 0.65` |
| Ownership entropy | `>= 0.95` | `>= 0.95` |
| Blockage/focus overlap | `>= 0.60` | `>= 0.60` |
| Cross-group close-encounter fraction | `>= 0.80` | `>= 0.80` |
| Target-target separation | `>= 14 m` | `>= 9 m` |
| Sensor-target separation | `>= 30 m` | `>= 30 m` |

The convoy blackout limits intentionally retain the earlier conservative
upper bound rather than using the near-zero development values as a tuned
threshold.  The new overlap limit reflects the safe offset-corridor task:
M24 spends a larger fraction in single-formation support, while still
requiring at least `40%` multi-formation target-time and at least one
ownership handover per target.

## Preregistered v5 cross-scale gates

Cross-scale comparison is evaluated only after both absolute realizations
pass.  For each shared seed index:

| Absolute M24/X36 difference or ratio | Convoy | Relay |
|:--|--:|--:|
| Global blackout difference | `<= 0.010` | `<= 0.010` |
| Focus blackout difference | `<= 0.010` | `<= 0.015` |
| Worst-target blackout difference | `<= 0.030` | `<= 0.050` |
| Longest-blackout difference | `<= 4` steps | `<= 8` steps |
| Single-formation fraction difference | `<= 0.15` | `<= 0.08` |
| Multi-formation fraction difference | `<= 0.15` | `<= 0.08` |
| X36/M24 focus-load ratio | `0.95--1.10` | `0.85--1.15` |
| Normalized-handover difference | `<= 0.20` | `<= 0.15` |
| Ownership-entropy difference | `<= 0.03` | `<= 0.03` |
| Blockage-overlap difference | `<= 0.05` | `<= 0.05` |

## Unopened validation manifest

The v5 held-out geometry validation uses exactly these twenty seeds:

```text
[401, 409, 419, 431, 443, 457, 467, 479, 487, 499,
 509, 521, 541, 557, 569, 577, 587, 599, 607, 617]
```

This creates `80` absolute realizations and `40` paired scale comparisons.
The decision rule is all-pass.  The seed set, gates, scene digests and
evaluation code must be committed before any result is opened.  A failure
creates a new scene version and a new unopened seed set; it does not permit
editing a threshold and reusing these observations.

Even a complete geometry-validation pass does not authorize tracking.  It
only permits a later source-frozen tracking protocol for radial, convoy and
relay.  Crossing remains excluded from averages and main claims.

## Artifacts

The Markdown report is committed with this assessment.  Its MAT file and raw
log remain locally available under the experiment ignore policy.

| Artifact | SHA-256 |
|:--|:--|
| `DYNAMIC_TOPOLOGY_SCENE_DIFFICULTY_20260803_141400.md` | `5eefd2202f9c7666218c3892982186e6bfd4fdb7f1419b2db7afe24c26329c99` |
| `DYNAMIC_TOPOLOGY_SCENE_DIFFICULTY_20260803_141400.mat` | `f42520cc5da17e168e53f150a5753dba3a57dd07c45058022b2b04e6fa8465ea` |
| `MULTISTYLE_V4_4X5_86D0724.log` | `87c28a66a9b79701c6a054c00652db042d9d3b6adf3f6fde54a50d7517d4fb41` |
