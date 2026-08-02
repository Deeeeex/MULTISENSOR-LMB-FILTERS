# V37 X36 runtime-fingerprint discovery

- Contract / generation commit: `formation-staggered-recovery-x36-v37-preflight-v3 / 58492d918a67a4c27357bf1759740f4f86ca4561`
- Tracked dirty / untracked source: `0 / 0`
- M24 screen SHA-256: `af314a31799db440a245026246f7690cc6bf1ce028e141bdbcb0f840910abae9`
- X36 scene SHA-256: `366274818ba6de2516678586bc6a9365bf7e100acdc1177d8615308bf63032ac`
- Registered executable-source manifest SHA-256: `c7c5a6cad8ed576f881cdd62794bcfbd6f63c8bafcfd3c1b26ffe2ca3f239908`
- Publication completion marker: `RUN/GA/dynamic_topology/evidence/formation_value_v37/x36_source_v3/fingerprint_discovery/FORMATION_STAGGERED_RECOVERY_X36_V37_V3_FINGERPRINT_DISCOVERY.mat.complete` (`formation-staggered-recovery-source-publication-v1`)
- Preset / seed / anchor times: `x36-formation-fov / 211 / [72 100 128]`
- Tracking scored / truth used: `0 / 0`
- Fingerprint discovery mode: `1`
- Eligible pair proposals: `0 / 3`
- Tracking pairs authorized by this artifact: `0`

The MAT/report pair is incomplete evidence unless the completion marker exists and reproduces both file hashes. The marker is published last.

## t=72, blockage pair [1 2]

- Cache SHA-256: `feffe13d789a3698ce29bc895793b460219396dad6a3fdbc3d0b570cf701ba25`
- Reference runtime fingerprint SHA-256: `2d3ff282f22946283e7a69fbaebbdfe68ddce2b3761e7bccd999d0ca2fcd15cc`
- Candidate runtime fingerprint SHA-256: `9a9930347c3c5f176c4bc0b3fd7529d8a6ffd05fdbc6c1cb3516c4fd49204eab`
- Initial action / retained formations: `53 / [3 5 6]`
- Initial reference / selected risk: `2.012962401694 / 2.085068466551`
- Initial retention risk / message saving: `0.00177581167365 / 3`
- Initial control / reference replay / candidate replay: `127.34 / 144.02 / 517.85 s`
- Reference attempted bytes: `[3522096 3637680 3745776]`
- Candidate attempted bytes: `[3332184 3581032 3868936]`
- Reference posterior/link adaptation: `0 / 0`; candidate: `1 / 1`
- Observable-context sanitizer passed (reference/candidate): `1 / 1`
- Explicit staggered releases / pair eligible: `0 / 0`

| Time | Debt fractions | Requested | Retained | Released | Stagger | Reserve | Attempted bytes |
|--:|:--|:--|:--|:--|:--:|:--:|--:|
| 72 | `[0.012462 0.053099 0.084357 0.055445 0.035509 0.02301]` | `[3 5 6]` | `[3 5 6]` | `[]` | 0 | 1 | 3332184 |
| 73 | `[0.011536 0.069454 0.066803 0.058937 0.039209 0.013932]` | `5` | `5` | `[]` | 0 | 1 | 3581032 |
| 74 | `[0.011793 0.060286 0.065394 0.051535 NaN 0.011722]` | `2` | `2` | `[]` | 0 | 1 | 3868936 |

## t=100, blockage pair [3 4]

- Cache SHA-256: `41f86ef83c70c98f39b8befa219067b201487dd99c2d6e88c628f5323babc306`
- Reference runtime fingerprint SHA-256: `712198520a745ca98089a053b2a3391b3280bf35c173f39344af98f3972a3425`
- Candidate runtime fingerprint SHA-256: `c296d38b885b9e66ab0be2fabf9696f87e908bc38b70af92619cdaf1a8f213a1`
- Initial action / retained formations: `52 / [1 2 5 6]`
- Initial reference / selected risk: `2.405047919256 / 2.579748613818`
- Initial retention risk / message saving: `1.81349050819e-05 / 4`
- Initial control / reference replay / candidate replay: `128.04 / 141.43 / 501.69 s`
- Reference attempted bytes: `[3080064 2897808 2681280]`
- Candidate attempted bytes: `[2861728 2713960 2759840]`
- Reference posterior/link adaptation: `0 / 0`; candidate: `1 / 1`
- Observable-context sanitizer passed (reference/candidate): `1 / 1`
- Explicit staggered releases / pair eligible: `1 / 0`

| Time | Debt fractions | Requested | Retained | Released | Stagger | Reserve | Attempted bytes |
|--:|:--|:--|:--|:--|:--:|:--:|--:|
| 100 | `[0.062804 0.073766 0.018745 0.082739 0.095173 0.068406]` | `[1 2 5 6]` | `[1 2 5 6]` | `[]` | 0 | 1 | 2861728 |
| 101 | `[0.056507 0.058125 0.01862 0.067167 0.1039 0.052045]` | `[1 2 4 5 6]` | `[1 2 4 5]` | `6` | 1 | 1 | 2713960 |
| 102 | `[NaN NaN 0.018148 0.077624 NaN 0.049792]` | `[4 6]` | `[4 6]` | `[]` | 0 | 1 | 2759840 |

## t=128, blockage pair [5 6]

- Cache SHA-256: `5f18f7cffeab5577949de72859f04b9619aacc98d8f7ac26bcd3998cfa29c8b8`
- Reference runtime fingerprint SHA-256: `afff6228ef68c3d3d7c7f51c18e67884807e448f7edf126d5a670dd3bb570f81`
- Candidate runtime fingerprint SHA-256: `6ff4e90f72563d410c7f3bfe60363c08d6d34ccd919dbf242966664f31b02881`
- Initial action / retained formations: `44 / [1 2 4 6]`
- Initial reference / selected risk: `1.098703159664 / 1.135072434932`
- Initial retention risk / message saving: `0.000238116158587 / 4`
- Initial control / reference replay / candidate replay: `124.56 / 119.96 / 467.35 s`
- Reference attempted bytes: `[2106480 1997376 1975248]`
- Candidate attempted bytes: `[1949704 1884000 1954304]`
- Reference posterior/link adaptation: `0 / 0`; candidate: `1 / 1`
- Observable-context sanitizer passed (reference/candidate): `1 / 1`
- Explicit staggered releases / pair eligible: `0 / 0`

| Time | Debt fractions | Requested | Retained | Released | Stagger | Reserve | Attempted bytes |
|--:|:--|:--|:--|:--|:--:|:--:|--:|
| 128 | `[0.050752 0.035647 0.039283 0.055885 0.019436 0.036723]` | `[1 2 4 6]` | `[1 2 4 6]` | `[]` | 0 | 1 | 1949704 |
| 129 | `[0.052946 0.033118 0.041091 0.053398 0.017231 0.020503]` | `[1 3 4]` | `[1 3 4]` | `[]` | 0 | 1 | 1884000 |
| 130 | `[NaN 0.036938 0.043644 NaN 0.019461 0.035336]` | `[3 6]` | `[3 6]` | `[]` | 0 | 1 | 1954304 |

## Method boundary

Each state freezes two distinct H=3 runtime traces. The reference uses the registered fixed counter-clockwise routing rule and is required not to adapt to posterior content or current link probabilities. The candidate recomputes the retention-debt control from the current posterior, current drop-probability page, current sensor positions, and past topology/update history only. Neither arm can receive target truth, complete scenario trajectories, link realizations, future drop-probability pages, future sensor trajectories, or future measurements through the policy callback. The posterior is schema-projected, and every remaining field uses a primitive container with a current-time shape contract. Only t=100 performs an explicit mature-formation release inside H=3; t=72 and t=128 rotate the protected formations. Staggered release is therefore a state-dependent behavior of the broader debt-aware protection-and-rotation mechanism.

## Decision

This source-only run discovers six complete runtime fingerprints: one reference and one candidate trace for each of the three states. It authorizes no tracking outcome. The hashes must first be frozen in source and reproduced from a later clean commit.

## Outcome-permit boundary

This source artifact authorizes no tracking run. A later immutable permit must bind this file hash to the three eligible pair records. Only that permit may authorize exactly one two-arm run per pair.

## Evidence boundary

V37 opens only the registered X36 formation-FoV seed-211 reference trajectory at t=72, 100, and 128. These anchors were fixed before any X36 posterior was generated and cover the three registered blockage-pair windows symmetrically. Scene geometry and every controller decision are audited through a structurally sanitized current-observable policy context. Both the fixed reference and causal candidate complete H=3 runtime traces must be frozen and reproduced from a later clean commit. X36 tracking, GNN, X48, reserved seeds, and validation remain sealed.
