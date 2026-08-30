# V190 alternative F2 candidate headroom

## Question

Did X36 formation 2 fail because one repair was too weak, or because V188
collapsed roughly one hundred executable source-label candidates to the wrong
top-one action before finite-horizon value could be evaluated?

## Paired teacher

- Scene: `x36-formation-fov`
- Seed / window: `211`, `t=72`, `H=3`
- Base method: V99 causal positive-net admission
- Repair page: first page only
- Target formation: 2
- Alternative candidate: source 19, label `[13,12]`
- Update: charged ideal-delivery hard replacement

## Result

| Metric | Static reference | V99 base | Alternative F2 teacher |
|:--|--:|--:|--:|
| Mean E-OSPA | 85.970277 | 83.561598 | 82.840396 |
| Mean RMSE | 57.902417 | 58.288297 | 54.371287 |
| E-OSPA gain vs static | 0 | +2.802% | +3.641% |
| RMSE gain vs static | 0 | -0.666% | +6.098% |
| Consensus gain vs static | 0 | +5.149% | +7.164% |
| Attempted-byte saving vs static | 0 | +6.550% | +5.161% |

Formation E-OSPA gains are
`[+1.626%, +7.836%, +2.399%, +4.007%, +5.652%, +0.000%]`.
Formation RMSE gains are
`[-0.069%, +62.290%, -1.489%, +1.818%, -1.000%, +0.000%]`.

Relative to V99, the alternative action improves mean E-OSPA by `0.863%`,
mean RMSE by `6.720%` and consensus by `2.124%`, at a charged cost of
`147,408 B`.  It converts the previously binding formation-2 RMSE gap from
`-14.198%` to `+62.290%`.

## Decision

The F2 failure was a candidate-selection failure, not evidence that formation
2 is intrinsically unrepairable.  V190 must preserve a diverse source-label
bank until finite-horizon valuation.  The result does not yet authorize a
method claim: mean E-OSPA remains below the development threshold, formations
1, 3 and 5 retain negative RMSE changes, delivery is idealized, and the
candidate key was teacher-forced on one opened window.

Next, compare this same action under residual label-wise KLA and find the F5
candidate.  Only after the per-action mechanisms are attributable should they
be combined by the budget projector.
