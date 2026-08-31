# V202 F2+F3 observation-handover finding

## Corrected recursive result

The corrected low-existence eligibility rule allows all planned F3 actions to
execute in the X36 `seed=211, t=72, H=8` teacher.  The sequence applies the
verified F2 label-KLA action at `t=72`, then sends the formation-common F3
label `[19,13]` at `t=76`, `t=78`, and `t=79`.  Every action multicasts one
complete Bernoulli GM density and applies receiver-specific residual label
KLA with source weight `0.5`.

| Metric | Static full payload | V99 no repair | F2+F3 handover |
|:--|--:|--:|--:|
| Mean E-OSPA | 84.037151 | 79.451115 | 78.032340 |
| Static-relative E-OSPA gain | 0 | +5.457% | +7.145% |
| Mean RMSE | 59.967347 | 62.172152 | 56.738264 |
| Static-relative RMSE gain | 0 | -3.677% | +5.385% |
| Window consensus gain | 0 | +8.803% | +11.383% |
| Terminal consensus gain | 0 | -- | +16.603% |
| Attempted-byte saving | 0 | +5.423% | +4.004% |

Relative to V99, residual repair improves E-OSPA by `1.786%`, RMSE by
`8.740%`, and consensus by `2.828%`.  It spends `405,384 B` above V99 but
retains a fully charged `4.004%` saving relative to static.

## The F3 mechanism is closed

The earlier F2-only V199 result left F3 at `-29.799%` RMSE and `+7.711%`
E-OSPA relative to static.  Corrected formation-level handover changes F3 to
`+5.795%` RMSE and `+9.549%` E-OSPA.  Thus the large F3 tail was neither a
general formation failure nor evidence that label KLA was ineffective.  It
was a low-existence active-label handover problem hidden by the old
`r >= 0.5` candidate gate.

The improvement is not explained by suppressing difficult tracks.  Mean
cardinality error changes from `5.770833` under V99 to `5.677083` after the
repairs, while E-OSPA and RMSE improve on every page of the eight-step window
relative to V99.

## Remaining binding tails

Formation-level gains relative to static are:

| Formation | F1 | F2 | F3 | F4 | F5 | F6 |
|:--|--:|--:|--:|--:|--:|--:|
| E-OSPA gain | -0.931% | +12.634% | +9.549% | +8.970% | +12.496% | -0.021% |
| RMSE gain | +1.847% | +55.486% | +5.795% | +1.789% | -2.561% | -0.749% |

The next method question is therefore no longer F3.  F6 has already been
identified as a useful precision-refresh action; F5 is now the largest RMSE
tail, and F1 remains a small set-level tail.  These modes should be repaired
without weakening the four aggregate gains above.

## Current-best comparison

V187 remains the balanced development best because its E-OSPA `74.678760`,
RMSE `53.540189`, and weakest-formation tails are better.  The F2+F3 sequence
is materially more communication-efficient (`4.004%` versus `0.160%` saving)
and improves window/terminal consensus more, so it is the stronger foundation
for a scalable residual-payload policy but does not replace the main-document
current-best row.

## Evidence boundary

Formation, source, label, and page identifiers are opened teacher routing
keys.  Delivery is ideal but every synopsis and payload is charged.  This
single X36 window establishes recursive mechanism headroom only; it is not an
online selector, cross-seed evidence, or a paper-facing validation result.
