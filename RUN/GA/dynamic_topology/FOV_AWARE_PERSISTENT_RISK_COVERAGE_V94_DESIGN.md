# V94 FoV-aware persistent risk coverage

## Question

The historical V65 result is the only method family that produced paired
M24 and X36 gains above 5%, but those runs used the old
`support-renormalized` missing-label receiver semantics. V94 asks whether the
same observable three-step schedule still beats a matched static route after
switching to `fov-aware-censored` semantics.

## Matched baseline

Every arm starts from the same cached posterior and uses the same scenario,
seed, measurements, link uniforms, filter RNG, three-step horizon, physical
reference graph and communication constraints. The baseline is the fixed
counter-clockwise static route with full payload consumption. The candidate
uses the frozen V65 observable risk rule to select formations and applies the
selection for all three steps.

This is a development headroom comparison, not yet a full-episode controller.
If it passes, the next full comparison must include both this canonical static
tree and a best-feasible static route frozen from the same action space.

## Frozen gate

The four already-opened radial anchors are M24 t=104/124 and X36 t=72/100.
Each anchor must achieve at least 5% mean E-OSPA improvement over its paired
static baseline. Worst-sensor E-OSPA, minimum-formation E-OSPA, window and
terminal consensus, and attempted bytes must not regress. All four anchors
must pass; otherwise the V65 persistent-protection family is closed under the
corrected receiver semantics.

No threshold is tuned after outcomes are opened, and no held-out or validation
claim is authorized.
