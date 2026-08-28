# V140 M24 finding: bidirectional cardinality matching erases useful targets

V140 fails the registered M24 gate and remains repository-only.  Against the
frozen paired reference, its intervention E-OSPA gain is `+0.765%`, compared
with `+5.614%` for V139.  Full-window gain is `+0.070%`, minimum sensor gain
is `-0.003%`, exact reentry is preserved, and the charged dual payload still
adds `89.182%` attempted bytes.  No X36 V140 run is authorized.

## Decisive attribution

V140 requested 32 reference-cardinality projections.  Seven were subsequently
overwritten by the inherited predictive fallback, leaving 25 effective
projections, all on intervention pages.  Every effective projection worsens
E-OSPA relative to V139; their mean increase is `19.8266`.  In all 25 cells,
the exact-reference cardinality is smaller than the hybrid cardinality:
`k_R-k_H` ranges from `-1` to `-6`.  The projection therefore deletes labels
that account for most of M24's positive intervention gain.

The direction of cardinality disagreement provides a threshold-free split:

| Frozen evidence | `k_R > k_H`, not already covered by predictive fallback | Outcome of those cells |
|:--|--:|:--|
| M24 V140/V139 pair | 0 | no undercount cell to repair |
| X36 V139 diagnostic | 6 | exactly all six severe negative cells |

Across the X36 diagnostic, 270 protected cells have finite hybrid/reference
cardinalities.  Only seven have `k_R > k_H`; one is already replaced by the
inherited predictive fallback.  The remaining six all have `k_R=k_H+1`, all
have gain no greater than `-5%`, and no positive cell satisfies the same
condition.  Conversely, M24's 25 effective V140 changes all have `k_R<k_H`.

## Next method decision

V141 must be an undercount-only augmentation, not a bidirectional projection:

1. if `k_R <= k_H`, preserve the complete V139 output exactly;
2. if `k_R > k_H`, retain every existing hybrid label and state, then append
   the next highest-existence hybrid labels until the output reaches `k_R`;
3. never remove a hybrid label and never mutate either hidden posterior;
4. keep the inherited output-only predictive fallback after augmentation.

This rule uses no truth, future measurement, threshold sweep, or opened result
mask.  It is selected from the sign of an observable reference/hybrid
cardinality discrepancy.  M24 is expected to be exactly identical to V139;
that identity must still be verified by a frozen rerun before X36 is opened.
