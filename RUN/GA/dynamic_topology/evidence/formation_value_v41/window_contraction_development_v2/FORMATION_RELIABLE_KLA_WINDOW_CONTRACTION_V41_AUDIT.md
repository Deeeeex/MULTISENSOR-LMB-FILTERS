# V41 M24/X36 window-contraction audit

## Evidence identity

- Generation commit: `4c23802c4e6a68650904db1f4ef6ba2327b3a6c7`
- Generation worktree dirty: `false`
- Protocol SHA-256:
  `88354e5ab93b168d7da617f9dbfed5e5e9f16cb0d6cccae0f65b68126832662a`
- Result SHA-256:
  `df6875d49e6524e97fd734fd11ecc23e5362275961202c2457e63293b3b3a2f9`
- Presets: `m24-formation-fov`, `x36-formation-fov`
- Seeds: `41`, `43`, `47`, `53`
- Evidence class: compact geometry-only development diagnostic
- Formal runtime-observable boundary: not passed
- Tracking, validation and paper-performance permits: not issued

The stored result hash was recomputed after removing its own hash field and
matched exactly.  The compact v2 artifact was also compared with the preserved
full v1 artifact at generation commit `17510cab97ee38ad4bfafadbd753c3d5e7064190`.
Every shared horizon, availability flag, reference factor and candidate factor
delta matched exactly; the maximum numerical difference was zero.  Compaction
therefore changed storage only, not the reported evidence.

All eight cases calibrated successfully.  The artifact records no truth,
target, measurement, posterior or realized-delivery input.

## Aggregate result

| Scale | Reference horizon | Reference rho, mean | Improving candidates | Stable best candidate | Mean delta rho | Relative change |
|:--|--:|--:|--:|:--|--:|--:|
| M24 | 31 | 0.885206799 | 0 / 16 | none; F4 is least harmful | +0.012570651 for F4 | +1.4201% |
| X36 | 95 | 0.888270649 | 4 / 24 | suspend F6 in all four seeds | -0.004471647 | -0.5034% |

The M24 reference factor ranged from `0.884896657` to `0.885369153`.
Every one of the four candidate formations worsened the operational
`renormalize` factor in every seed.  Even the least harmful formation, F4,
worsened the factor by `0.012531748--0.012619940`.

The X36 reference factor ranged from `0.886823725` to `0.889080019`.  F6 was
the only improving action in every seed, with delta rho in
`[-0.004481226, -0.004463456]`; all other formations were neutral-to-worse.
The same action direction also held under the non-operational `self`
sensitivity calculation.

The auxiliary single-event contraction gain remained numerically unusable:
its mean was approximately `5.06e-6` on M24 and `9.24e-9` on X36.

## Interpretation

This is a stable structural asymmetry, not a tracking result.  It establishes
that the exact mean-square factor is nonconstant and can identify one
repeatable X36 action, while simultaneously falsifying the claim that the
same residual-bundle suspension is structurally favorable on M24.

The formation identity is suspiciously specific: F6 is favored in every X36
seed, whereas no M24 formation is favored.  Before treating this as a method
signal, the next diagnostic must test complete formation-block/node-coordinate
permutations and the convoy, crossing and relay scene styles.  If the favored
action follows the deterministic Euler-tour ordering instead of a physical
formation role, it is a routing-construction artifact.

The communication reduction is also small in the calibrated window: the
candidate changes only the first page, saving two attempted messages out of
`6,840` on X36.  The structural improvement therefore does not yet establish
a useful communication-estimation tradeoff.

## Decision

1. Accept V41 as an implementation-checked structural development package.
2. Keep M24/X36 tracking sealed because posterior disturbances are unbounded
   and the runner is not a formal runtime-observable policy manifest.
3. Run index-equivariance and multistyle diagnostics before using F6 as a
   controller feature.
4. Develop a proper-metric local posterior disturbance bound; use the exact
   propagation factor and suffix coefficients as analytic features or
   constraints for the later value model.
