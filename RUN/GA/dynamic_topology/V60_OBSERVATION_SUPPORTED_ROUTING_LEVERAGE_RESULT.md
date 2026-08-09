# V60 observation-supported routing leverage result

V60 tests whether the V58 rescue signal was inflated by labels that changed
under fusion but lacked current measurement support at the receiver.  It
recomputes the observable signal on 29 cached M24 states and joins only the ten
already-opened H=3 outcomes after feature construction.

| Observable signal | Pearson correlation with known strict gain | Strong/weak separation |
|:--|--:|:--:|
| Raw rescued existence | `+0.9134` | pass |
| Positive-supported rescue | `+0.9476` | pass |
| Association-weighted rescue | `+0.9554` | pass |
| Supported decision crossings | `+0.9267` | pass |

The false-positive convoy state at `t=40` falls from `18.562%` raw rescue to
`0.501%` supported rescue and has only `+0.054%` strict tracking gain.  In
contrast, radial `t=104` retains `8.493%` supported rescue and `7.544%`
association-weighted rescue with `+10.394%` known gain.  The correction also
selects fresh convoy times `[56, 80, 96]` for later frozen evaluation.

## Decision

Current receiver measurement support is a necessary feature for deciding
which sender-label input may be harmful.  It improves attribution, but it does
not by itself define a deployable scalar event trigger.  V61 therefore uses
the support signal inside receiver--sender--label counterfactual action
construction and keeps a deterministic reference fallback.

This is development attribution only.  It opens no new tracking outcome,
X36 result, model training, held-out claim, or validation claim.
