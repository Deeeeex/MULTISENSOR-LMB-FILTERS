# V42 formation-route index-equivariance diagnostic

- Contract: `formation-reliable-kla-index-equivariance-development-result-v1`
- Protocol SHA-256: `7572ebcc50e3080ac4e67a3e6133f52720463248535a15c85e65e0dbbed80ed4`
- Base V41 protocol SHA-256: `88354e5ab93b168d7da617f9dbfed5e5e9f16cb0d6cccae0f65b68126832662a`
- Generation commit: `bc6eae839308623aee83f49beef73f9d5b3b2bae` (dirty: `0`)
- Resolved generation repo root: `/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v42-structural-equivariance`
- MAT artifact: `/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v42-structural-equivariance/RUN/GA/dynamic_topology/evidence/formation_value_v42/index_equivariance_development_v1/FORMATION_RELIABLE_KLA_INDEX_EQUIVARIANCE_V42_DEVELOPMENT_V1.mat`
- Result SHA-256: `eae2c7f871f60740e55f376c172fd8523e593641811fb034e67328abd999e2ad`

- Registered Cartesian batch complete: `1`; exploratory subset only: `0`.
- Clean-commit reconstruction identity captured: `1`.

Each row is produced after reordering complete formation blocks in node coordinates and mapping the resulting route back to the original physical node coordinates. Formation labels, membership, geometry, reliability and route history are unchanged.

## Case summary

| Preset | Seed | N | F | Equivariant? | Unique reference routes | Max ref weight diff | Max ref rho diff | Best physical IDs | Best order positions | Best delta range | Pattern | Primary failure layer |
|:--|--:|--:|--:|:--:|--:|--:|--:|:--|:--|:--|:--|:--|
| m24-formation-fov | 41 | 24 | 4 | 0 | 4 | 0.05 | 0.000884 | 4,1,2,3 | 4,4,4,4 | [+0.0125462, +0.0125989] | formation-order-position | reference-route-or-weight |
| m24-formation-fov | 43 | 24 | 4 | 0 | 4 | 0.05 | 0.000421 | 4,1,2,3 | 4,4,4,4 | [+0.0125406, +0.0125855] | formation-order-position | reference-route-or-weight |
| m24-formation-fov | 47 | 24 | 4 | 0 | 4 | 0.05 | 0.000135 | 4,1,2,3 | 4,4,4,4 | [+0.0126062, +0.0126251] | formation-order-position | reference-route-or-weight |
| m24-formation-fov | 53 | 24 | 4 | 0 | 4 | 0.05 | 0.000246 | 4,1,2,3 | 4,4,4,4 | [+0.0125013, +0.0125317] | formation-order-position | reference-route-or-weight |
| x36-formation-fov | 41 | 36 | 6 | 0 | 6 | 0.05 | 0.000813 | 6,1,2,3,4,5 | 6,6,6,6,6,6 | [-0.00448691, -0.00446346] | formation-order-position | reference-route-or-weight |
| x36-formation-fov | 43 | 36 | 6 | 0 | 6 | 0.05 | 0.000502 | 6,1,2,3,4,5 | 6,6,6,6,6,6 | [-0.00448315, -0.00446881] | formation-order-position | reference-route-or-weight |
| x36-formation-fov | 47 | 36 | 6 | 0 | 6 | 0.05 | 0.00109 | 6,1,2,3,4,5 | 6,6,6,6,6,6 | [-0.00448448, -0.00446401] | formation-order-position | reference-route-or-weight |
| x36-formation-fov | 53 | 36 | 6 | 0 | 6 | 0.05 | 0.002 | 6,1,2,3,4,5 | 6,6,6,6,6,6 | [-0.00448711, -0.00445517] | formation-order-position | reference-route-or-weight |

## Shift-level evidence

| Preset | Seed | Shift | Formation order | H | Ref rho | Same restored ref adjacency? | Best physical ID | Position | Best delta | Improving IDs |
|:--|--:|--:|:--|--:|--:|:--:|--:|--:|--:|:--|
| m24-formation-fov | 41 | 0 | 1,2,3,4 | 31 | 0.8853691526 | 1 | 4 | 4 | +0.0125902750 | none |
| m24-formation-fov | 41 | 1 | 2,3,4,1 | 31 | 0.8844854861 | 0 | 1 | 4 | +0.0125838844 | none |
| m24-formation-fov | 41 | 2 | 3,4,1,2 | 31 | 0.8850815021 | 0 | 2 | 4 | +0.0125989302 | none |
| m24-formation-fov | 41 | 3 | 4,1,2,3 | 31 | 0.8856106011 | 0 | 3 | 4 | +0.0125462430 | none |
| m24-formation-fov | 43 | 0 | 1,2,3,4 | 31 | 0.8853129426 | 1 | 4 | 4 | +0.0125406429 | none |
| m24-formation-fov | 43 | 1 | 2,3,4,1 | 31 | 0.8851468140 | 0 | 1 | 4 | +0.0125839229 | none |
| m24-formation-fov | 43 | 2 | 3,4,1,2 | 31 | 0.8848921735 | 0 | 2 | 4 | +0.0125754690 | none |
| m24-formation-fov | 43 | 3 | 4,1,2,3 | 31 | 0.8854750765 | 0 | 3 | 4 | +0.0125854849 | none |
| m24-formation-fov | 47 | 0 | 1,2,3,4 | 31 | 0.8848966574 | 1 | 4 | 4 | +0.0126199401 | none |
| m24-formation-fov | 47 | 1 | 2,3,4,1 | 31 | 0.8849365773 | 0 | 1 | 4 | +0.0126163691 | none |
| m24-formation-fov | 47 | 2 | 3,4,1,2 | 31 | 0.8850314117 | 0 | 2 | 4 | +0.0126250665 | none |
| m24-formation-fov | 47 | 3 | 4,1,2,3 | 31 | 0.8849270444 | 0 | 3 | 4 | +0.0126062097 | none |
| m24-formation-fov | 53 | 0 | 1,2,3,4 | 31 | 0.8852484418 | 1 | 4 | 4 | +0.0125317478 | none |
| m24-formation-fov | 53 | 1 | 2,3,4,1 | 31 | 0.8854608248 | 0 | 1 | 4 | +0.0125012554 | none |
| m24-formation-fov | 53 | 2 | 3,4,1,2 | 31 | 0.8854945700 | 0 | 2 | 4 | +0.0125292041 | none |
| m24-formation-fov | 53 | 3 | 4,1,2,3 | 31 | 0.8851705912 | 0 | 3 | 4 | +0.0125309927 | none |
| x36-formation-fov | 41 | 0 | 1,2,3,4,5,6 | 95 | 0.8890800193 | 1 | 6 | 6 | -0.0044634563 | 6 |
| x36-formation-fov | 41 | 1 | 2,3,4,5,6,1 | 95 | 0.8890676069 | 0 | 1 | 6 | -0.0044869134 | 1 |
| x36-formation-fov | 41 | 2 | 3,4,5,6,1,2 | 95 | 0.8882665408 | 0 | 2 | 6 | -0.0044729186 | 2 |
| x36-formation-fov | 41 | 3 | 4,5,6,1,2,3 | 95 | 0.8892060185 | 0 | 3 | 6 | -0.0044686740 | 3 |
| x36-formation-fov | 41 | 4 | 5,6,1,2,3,4 | 95 | 0.8892702308 | 0 | 4 | 6 | -0.0044745286 | 4 |
| x36-formation-fov | 41 | 5 | 6,1,2,3,4,5 | 95 | 0.8885310974 | 0 | 5 | 6 | -0.0044698928 | 5 |
| x36-formation-fov | 43 | 0 | 1,2,3,4,5,6 | 95 | 0.8886718989 | 1 | 6 | 6 | -0.0044768899 | 6 |
| x36-formation-fov | 43 | 1 | 2,3,4,5,6,1 | 95 | 0.8891009164 | 0 | 1 | 6 | -0.0044748177 | 1 |
| x36-formation-fov | 43 | 2 | 3,4,5,6,1,2 | 95 | 0.8888432162 | 0 | 2 | 6 | -0.0044804041 | 2 |
| x36-formation-fov | 43 | 3 | 4,5,6,1,2,3 | 95 | 0.8888567925 | 0 | 3 | 6 | -0.0044688073 | 3 |
| x36-formation-fov | 43 | 4 | 5,6,1,2,3,4 | 95 | 0.8891742180 | 0 | 4 | 6 | -0.0044728257 | 4 |
| x36-formation-fov | 43 | 5 | 6,1,2,3,4,5 | 95 | 0.8886661263 | 0 | 5 | 6 | -0.0044831497 | 5 |
| x36-formation-fov | 47 | 0 | 1,2,3,4,5,6 | 95 | 0.8868237251 | 1 | 6 | 6 | -0.0044650161 | 6 |
| x36-formation-fov | 47 | 1 | 2,3,4,5,6,1 | 95 | 0.8879097913 | 0 | 1 | 6 | -0.0044675248 | 1 |
| x36-formation-fov | 47 | 2 | 3,4,5,6,1,2 | 95 | 0.8878351707 | 0 | 2 | 6 | -0.0044663813 | 2 |
| x36-formation-fov | 47 | 3 | 4,5,6,1,2,3 | 95 | 0.8875319547 | 0 | 3 | 6 | -0.0044698272 | 3 |
| x36-formation-fov | 47 | 4 | 5,6,1,2,3,4 | 95 | 0.8870106468 | 0 | 4 | 6 | -0.0044640126 | 4 |
| x36-formation-fov | 47 | 5 | 6,1,2,3,4,5 | 95 | 0.8865165612 | 0 | 5 | 6 | -0.0044844768 | 5 |
| x36-formation-fov | 53 | 0 | 1,2,3,4,5,6 | 95 | 0.8885069538 | 1 | 6 | 6 | -0.0044812256 | 6 |
| x36-formation-fov | 53 | 1 | 2,3,4,5,6,1 | 95 | 0.8894453098 | 0 | 1 | 6 | -0.0044754750 | 1 |
| x36-formation-fov | 53 | 2 | 3,4,5,6,1,2 | 95 | 0.8904547510 | 0 | 2 | 6 | -0.0044717252 | 2 |
| x36-formation-fov | 53 | 3 | 4,5,6,1,2,3 | 95 | 0.8905043677 | 0 | 3 | 6 | -0.0044784735 | 3 |
| x36-formation-fov | 53 | 4 | 5,6,1,2,3,4 | 95 | 0.8895518061 | 0 | 4 | 6 | -0.0044551743 | 4 |
| x36-formation-fov | 53 | 5 | 6,1,2,3,4,5 | 95 | 0.8890518559 | 0 | 5 | 6 | -0.0044871095 | 5 |

## Decision rule and claim boundary

- Required weight tolerance: `1e-12`; factor tolerance: `1e-12`.
- All requested cases equivariant over tested cyclic block orders: `0`; registered 8-case conclusion: `0`; any ordering confound: `1`.
- The diagnostic covers only `4/24` M24 and `6/720` X36 block orders and no within-formation sensor permutations; `physicalActionInterpretationAuthorized=false`.
- A failed case invalidates a physical-formation interpretation of its V41 action value. The reported primary failure layer identifies whether the route, candidate construction, horizon, or certificate pipeline must be corrected first.
- Passing removes only the tested cyclic block-order confound; it does not establish physical-action or tracking value.
- The full planned sensor and link-probability schedules were materialized. `formalRuntimeObservableBoundaryPassed=false`.
- No target, measurement, posterior content, realized delivery uniform or tracking outcome was used. M24/X36 tracking remains unauthorized.
