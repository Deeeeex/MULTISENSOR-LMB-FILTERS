# REVOKED — V37 X36 source-only preflight v2

This artifact was revoked on 2026-08-03 before any formal X36 outcome was
produced. The runtime policy context still exposed nested target trajectories
and future link realizations even though the controller reported that it did
not use truth. Consequently, this artifact does not establish the claimed
causal-input boundary and cannot authorize any tracking run. A new preflight
must remove those fields structurally and freeze both reference and candidate
runtime fingerprints.

- Contract / generation commit: `formation-staggered-recovery-x36-v37-preflight-v2 / 57f65d0b860755157a5af7d65be8073bea065574`
- Tracked dirty / untracked source: `0 / 0`
- M24 screen SHA-256: `af314a31799db440a245026246f7690cc6bf1ce028e141bdbcb0f840910abae9`
- X36 scene SHA-256: `366274818ba6de2516678586bc6a9365bf7e100acdc1177d8615308bf63032ac`
- Preset / seed / anchor times: `x36-formation-fov / 211 / [72 100 128]`
- Tracking scored / truth used: `0 / 0`
- Fingerprint discovery mode: `0`
- Eligible pair proposals: `3 / 3`
- Tracking pairs authorized by this artifact: `0`

## t=72, blockage pair [1 2]

- Cache SHA-256: `feffe13d789a3698ce29bc895793b460219396dad6a3fdbc3d0b570cf701ba25`
- Runtime fingerprint SHA-256: `fef95ee229bc6c4a86142ed72e3f6455d3dfac807a63b92ac2136bf905431f7a`
- Initial action / retained formations: `53 / [3 5 6]`
- Initial reference / selected risk: `2.012962401694 / 2.085068466551`
- Initial retention risk / message saving: `0.00177581167365 / 3`
- Initial control / runtime replay: `128.62 / 511.50 s`
- Explicit staggered releases / pair eligible: `0 / 1`

| Time | Debt fractions | Requested | Retained | Released | Stagger | Reserve | Attempted bytes |
|--:|:--|:--|:--|:--|:--:|:--:|--:|
| 72 | `[0.012462 0.053099 0.084357 0.055445 0.035509 0.02301]` | `[3 5 6]` | `[3 5 6]` | `[]` | 0 | 1 | 3332184 |
| 73 | `[0.011536 0.069454 0.066803 0.058937 0.039209 0.013932]` | `5` | `5` | `[]` | 0 | 1 | 3581032 |
| 74 | `[0.011793 0.060286 0.065394 0.051535 NaN 0.011722]` | `2` | `2` | `[]` | 0 | 1 | 3868936 |

## t=100, blockage pair [3 4]

- Cache SHA-256: `41f86ef83c70c98f39b8befa219067b201487dd99c2d6e88c628f5323babc306`
- Runtime fingerprint SHA-256: `8f71e1c1e37cfb7d63f1f481788f9fea2277f69940d0bca52f5878b7c47b7a4c`
- Initial action / retained formations: `52 / [1 2 5 6]`
- Initial reference / selected risk: `2.405047919256 / 2.579748613818`
- Initial retention risk / message saving: `1.81349050819e-05 / 4`
- Initial control / runtime replay: `131.31 / 515.59 s`
- Explicit staggered releases / pair eligible: `1 / 1`

| Time | Debt fractions | Requested | Retained | Released | Stagger | Reserve | Attempted bytes |
|--:|:--|:--|:--|:--|:--:|:--:|--:|
| 100 | `[0.062804 0.073766 0.018745 0.082739 0.095173 0.068406]` | `[1 2 5 6]` | `[1 2 5 6]` | `[]` | 0 | 1 | 2861728 |
| 101 | `[0.056507 0.058125 0.01862 0.067167 0.1039 0.052045]` | `[1 2 4 5 6]` | `[1 2 4 5]` | `6` | 1 | 1 | 2713960 |
| 102 | `[NaN NaN 0.018148 0.077624 NaN 0.049792]` | `[4 6]` | `[4 6]` | `[]` | 0 | 1 | 2759840 |

## t=128, blockage pair [5 6]

- Cache SHA-256: `5f18f7cffeab5577949de72859f04b9619aacc98d8f7ac26bcd3998cfa29c8b8`
- Runtime fingerprint SHA-256: `c39e67d450a697632a41b807272bfdd20ccb4b7695bca08e04dbb37668221067`
- Initial action / retained formations: `44 / [1 2 4 6]`
- Initial reference / selected risk: `1.098703159664 / 1.135072434932`
- Initial retention risk / message saving: `0.000238116158587 / 4`
- Initial control / runtime replay: `125.64 / 463.80 s`
- Explicit staggered releases / pair eligible: `0 / 1`

| Time | Debt fractions | Requested | Retained | Released | Stagger | Reserve | Attempted bytes |
|--:|:--|:--|:--|:--|:--:|:--:|--:|
| 128 | `[0.050752 0.035647 0.039283 0.055885 0.019436 0.036723]` | `[1 2 4 6]` | `[1 2 4 6]` | `[]` | 0 | 1 | 1949704 |
| 129 | `[0.052946 0.033118 0.041091 0.053398 0.017231 0.020503]` | `[1 3 4]` | `[1 3 4]` | `[]` | 0 | 1 | 1884000 |
| 130 | `[NaN 0.036938 0.043644 NaN 0.019461 0.035336]` | `[3 6]` | `[3 6]` | `[]` | 0 | 1 | 1954304 |

## Method boundary

All three states use the same posterior- and link-aware, retention-debt receding-horizon controller. Only t=100 performs an explicit mature-formation release inside H=3; t=72 and t=128 rotate the retained formations without invoking that release schedule. The X36 test therefore evaluates debt-aware protection and rotation as the general mechanism, with staggered release as one state-dependent behavior rather than a universal step.

## Decision

The numerical source traces reproduced, but the causal-input attestation was
invalid because forbidden fields remained reachable from the policy context.
The three pair proposals are therefore ineligible and this artifact is revoked.

## Outcome-permit boundary

This source artifact authorizes no tracking run and may not be bound by any
later permit. Only a newly generated causal preflight may open a replacement
permit.

## Evidence boundary

V37 opens only the registered X36 formation-FoV seed-211 reference trajectory at t=72, 100, and 128. These anchors were fixed before any X36 posterior was generated and cover the three registered blockage-pair windows symmetrically. Scene geometry and every controller decision are audited without tracking truth or future outcomes. X36 tracking remains sealed until the exact initial controls and complete H=3 runtime traces are frozen and reproduced from a later clean commit. GNN, X48, reserved seeds, and validation remain sealed.
