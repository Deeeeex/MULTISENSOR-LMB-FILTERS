# V139 X36 finding: positive formation means but localized extraction loss

V139 passes M24 but fails its frozen X36 mechanism gate.  The joint-scale
claim is therefore closed, budget-feasible representation work is not
authorized, and neither V139 result should be copied into the main progress
document.

## Frozen X36 result

- Intervention E-OSPA gain: **+4.901%** (required: at least +5%).
- Full-window / mature-window gain: **+0.453% / +0.000%**.
- Minimum sensor gain: **-0.161%**, below the registered `-0.01%`
  numerical tolerance.
- All six formation gains are positive; the minimum is **+0.234%**.
- Exact whole-formation reentry match: **100%**.
- Charged attempted-byte delta: **+89.403%**.
- The output-only predictive gate fires 65 times and restores reference
  E-OSPA at all of those node-pages.

## Mechanism interpretation

The scale failure is localized rather than common-mode.  Network page gains
over the ten intervention pages are
`[1.645, 4.881, 6.159, 6.008, 5.785, 6.920, 5.072, 5.442, 5.329, 1.787]%`.
The material negative cells concentrate in formation 1, especially sensors
4--6 on pages 4--6, while every formation still improves on average.

Post-fusion relay-label fraction is only weakly related to cell E-OSPA gain,
so thresholding the fraction would remove many useful outputs.  A perfect
result-only repair of all negative cells raises intervention gain only to
`+5.044%`, leaving little room for a coarse node- or formation-wide fallback.

The remaining roughly `-9%` cell jumps could be caused by a nonlinear MAP
cardinality/component extraction boundary, but the frozen screen artifact does
not retain state-estimate cardinalities.  This is a hypothesis, not yet a
finding.

## Next bounded diagnostic

Run only the first ten X36 continuation pages with the frozen V139 method and
record, per node-page, the local, W-fused, R-fused, label-readout and final
output MAP cardinalities.  Do not change the method or gates.  Continue toward
a cardinality-consistent output rule only if those diagnostics explain the
localized negative cells without selecting broad positive regions.
