# V46 causal-repair full32 structural preflight

## Frozen identity

| Field | Value |
|---|---|
| Generation commit | `e719fd6475366400d462f1d65cdf3f4067068bbd` |
| Generation worktree | clean tracked source at launch; only ignored evidence outputs were created during the run |
| Protocol | `formation-causal-minimal-edit-v46-development-v1` |
| Protocol SHA-256 | `01516bc9b58437b4c7436c1f160706cb313a6886b55b15f758f84fd78b64f848` |
| Result canonical SHA-256 | `f26e0a96c6df78c32500ed05b1cf7f8a70e9b58ee83b3998181141f465aff42a` |
| MAT file SHA-256 | `1bbea7c1dd7e6aae242c977fda55942abcd36cdab890bc830e52b1753f4af7d7` |
| Log file SHA-256 | `5a146b67872f39918556102fe16949f30c8b2c87e144c62447a1483ec500922b` |

The MAT envelope contains exactly `result` and `runMetadata`.  The stored
result canonical hash was recomputed from the payload after loading and
matched exactly.  `runMetadata` binds the result to the generation commit,
clean-launch flag, case count, and result hash.

## Gate result

| Check | Result |
|---|---:|
| Registered cases completed | 32 / 32 |
| Full-horizon pages audited | 5,120 / 5,120 |
| Rolling B4 products audited | 5,024 |
| Aligned four-step cycles audited | 1,280 |
| Full V43 sensor-level composition | pass in every page |
| Current physical-edge support | pass in every page |
| Positive fusion-weight support equals executed topology | pass in every page |
| Actual rolling-B4 sensor and formation unions | strongly connected in every window |
| Reference message count | exactly `2N` on every page |
| Synchronized B4 count pattern | exactly `[2N, N, N, N]` in every aligned cycle |
| Aligned attempted-message saving | exactly 37.5% |
| Posterior, truth, measurement, future page, or realized delivery draw read by the structural preflight | none |
| Full32 structural gate | **pass** |

## Per-scene result

| Preset | Cases | Repair cases | Repair pages | First repair | Largest removal on one page | Windows with centered spectral norm above 1 | Worst centered spectral norm | Median centered spectral norm | Worst centered-row L2 | Maximum projection work units |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `m24-formation-fov` | 4 | 0 | 0 | — | 0 | 0 / 628 | 0.954623 | 0.948705 | 0.466690 | 121 |
| `m24-formation-fov-convoy` | 4 | 0 | 0 | — | 0 | 0 / 628 | 0.990949 | 0.985702 | 0.466392 | 73 |
| `m24-formation-fov-relay` | 4 | 0 | 0 | — | 0 | 0 / 628 | 0.991649 | 0.987279 | 0.464700 | 73 |
| `m24-formation-fov-crossing` | 4 | 0 | 0 | — | 0 | 0 / 628 | 0.990949 | 0.986032 | 0.470862 | 73 |
| `x36-formation-fov` | 4 | 0 | 0 | — | 0 | 0 / 628 | 0.985765 | 0.976833 | 0.479082 | 181 |
| `x36-formation-fov-convoy` | 4 | 0 | 0 | — | 0 | 160 / 628 | 1.001702 | 0.995560 | 0.478953 | 133 |
| `x36-formation-fov-relay` | 4 | 0 | 0 | — | 0 | 160 / 628 | 1.002610 | 0.996677 | 0.477823 | 133 |
| `x36-formation-fov-crossing` | 4 | 4 | 14 | 157–158 | 3 | 72 / 628 | 1.002441 | 0.994533 | 0.481732 | 1,626 |

Only X36 crossing required a backbone edit.  Across its four seeds, 14 of
640 pages were repaired.  The per-seed edit traces were:

| Seed | Repair times | Removed registered pairs per repaired page | Added nonregistered pairs per repaired page |
|---:|---|---|---|
| 41 | 158–160 | `[1, 2, 3]` | `[1, 2, 3]` |
| 43 | 157–160 | `[1, 1, 2, 3]` | `[1, 1, 2, 3]` |
| 47 | 158–160 | `[1, 2, 3]` | `[1, 2, 3]` |
| 53 | 157–160 | `[1, 1, 1, 2]` | `[1, 1, 1, 2]` |

The maximum bounded-search work was 1,626 units, far below the registered
one-million-candidate fail-closed cap.  All no-repair pages reproduced the
registered formation graph exactly.

## Mixing interpretation

The one-cycle matrix audit must not be overstated.  The Dobrushin coefficient
of every four-page product was exactly 1.  The centered spectral norm was
below 1 in 4,632 of 5,024 windows and slightly above 1 in 392 X36 windows;
the global maximum was 1.002610.  A sparse jointly connected B4 sequence need
not be scrambling or contractive in a single four-step window, and a
non-normal stochastic product may have centered spectral norm above 1 even
when longer products converge.

Consequently this result establishes executable current-page repair,
physical support, rolling joint connectivity, and exact communication counts.
It does **not** establish single-cycle contraction, tracking non-inferiority,
delivered-byte saving, or a validation claim.  Longer-horizon product analysis
and the already frozen paired tracking gate remain necessary.
