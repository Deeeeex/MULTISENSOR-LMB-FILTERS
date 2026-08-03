# V42 formation-route index-equivariance audit

## Evidence identity

- Generation commit: `bc6eae839308623aee83f49beef73f9d5b3b2bae`
- Generation worktree dirty: `false`
- V42 protocol SHA-256:
  `7572ebcc50e3080ac4e67a3e6133f52720463248535a15c85e65e0dbbed80ed4`
- V41 base protocol SHA-256:
  `88354e5ab93b168d7da617f9dbfed5e5e9f16cb0d6cccae0f65b68126832662a`
- Result SHA-256:
  `eae2c7f871f60740e55f376c172fd8523e593641811fb034e67328abd999e2ad`
- Registered batch: complete `2 x 4 = 8` Cartesian cases
- Evidence class: geometry-only cyclic block-order diagnostic
- Formal runtime-observable boundary: not passed
- Physical-action, tracking and validation permits: not issued

The stored result hash was recomputed after removing its own hash field and
matched exactly.  The artifact was generated from the clean V42 worktree, its
critical executable paths and source-file hashes were recorded, and the full
registered preset/seed batch was present.

## Decisive result

| Scale | Failed cases | Restored reference routes per case | Best physical IDs across cyclic shifts | Best array position | Candidate sign |
|:--|--:|--:|:--|--:|:--|
| M24 | 4 / 4 | 4 | `F4,F1,F2,F3` | 4 in every shift | always worse than reference |
| X36 | 4 / 4 | 6 | `F6,F1,F2,F3,F4,F5` | 6 in every shift | always better than reference |

Every case failed at the `reference-route-or-weight` layer.  Reordering only
the complete formation blocks produced a different restored physical
reference route on every nonzero shift.  The maximum fusion-weight difference
was exactly `0.05`, the residual-route weight, showing that the changed object
was the low-weight residual input itself rather than a numerical eigensolver
artifact.

For M24, the best candidate always followed the fourth position in the stable
formation array.  Its factor delta remained positive, between approximately
`+0.01250` and `+0.01263`, so no sparse action improved contraction.

For X36, the improving candidate always followed the sixth position.  Its
factor delta remained negative, between approximately `-0.004487` and
`-0.004455`, but the physical formation carrying that value changed with every
cyclic shift.  F6 appeared stable in V41 only because F6 occupied the sixth and
last position in the original array order.

## Interpretation

The V41 X36 signal is a deterministic routing-construction artifact.  It does
not identify a physically special formation and cannot support a controller,
tracking experiment, or paper claim.  The same ordering mechanism also
explains why M24 repeatedly preferred F4 as the least harmful action.

The failure occurs before posterior modeling: `unique(...,'stable')`, an Euler
walk rooted at the first formation, first-neighbor tie resolution, and
index-based member partitioning change the residual Hamiltonian cycle when
the coordinate ordering changes.  Because the candidate removes an input
bundle from that arbitrary cycle, its apparent value is attached to a route
role, not to a formation.

This registered diagnostic covers all cyclic block orders but not all `F!`
block permutations, within-formation sensor permutations, or formation-label
renaming.  Those missing audits do not weaken the negative conclusion: one
valid pure-coordinate permutation counterexample is sufficient to disprove
index equivariance.  They remain required before a future replacement route
can pass.

## Decision

1. Reject the V41 F6 physical-action interpretation and keep all tracking
   permits closed.
2. Do not spend multistyle or posterior-tracking compute on the current
   index-dependent residual route.
3. Replace both the index-based local dominant route and the residual tour
   with a construction driven by observable physical features or reliability,
   with fail-closed ties and no label/index fallback.
4. Require full formation-block permutations, within-formation sensor
   permutations and formation-label renaming before evaluating structural
   gains of the replacement.
5. Only after this gate passes should the exact window certificate constrain a
   data-driven edge-value model and safe topology projection.
