# V208 formation-graph action-bank finding

## Scope

This is development evidence from the already opened X36 formation-FoV
trajectory at seed 211.  The action bank and every model input are built from
current posteriors, geometry, physical links, causal route history, and byte
accounting.  Truth is attached only after an action is frozen to score its
current-state effect.  The result is not recursive, held out, cross-seed, or
cross-scene evidence.

## Search-space preflight

The diverse truth-free shortlist reduces the supported-label KLA bank from a
mean of `3751.9` to `103.9` candidates per page (`2.769%` retained).  Despite
that reduction, all five routing keys later used by the V206 mechanism teacher
remain in the shortlist.  This establishes search-space coverage only; the
teacher keys are read after shortlisting and never enter the ranking features.

## Dense t=72 action values

The first complete block contains 108 shortlisted and executable actions.
All 108 fit the same-page communication credit after charging both the light
synopsis and the complete-label payload.

| Criterion | Count |
|:--|--:|
| Mean E-OSPA and RMSE both improve | 21 / 108 |
| Mean E-OSPA, RMSE, and consensus improve; byte credit remains positive | 15 / 108 |
| Above conditions plus positive receiver-formation E-OSPA/RMSE and no affected-sensor or network-worst regression | 5 / 108 |

The known V206 action `F2 <- S19, label [13,12]` is recovered without using
truth in construction.  Its one-step values are:

| Mean E-OSPA | Mean RMSE | Consensus | Net byte saving | Min affected E-OSPA | Min affected RMSE |
|--:|--:|--:|--:|--:|--:|
| +0.932% | +7.163% | +2.440% | +5.180% | +3.913% | +63.671% |

It is the strongest zero-tolerance tail-safe action in this block.  A second
source, `F2 <- S24` for the same label, has nearly identical tracking and
consensus value and slightly more byte credit, showing that the useful action
is not identified only by one memorized sensor key.

## Why a scalar mean reward is unsafe

The two largest mean tracking actions target F5 with the same `[13,12]` label.
They improve mean E-OSPA by about `0.96%` and mean RMSE by about `7.93%`, but
worsen consensus by about `0.44%` and worsen the minimum affected-sensor RMSE
by about `143%`.  A controller trained or ranked only on mean E-OSPA/RMSE would
prefer these actions over the V206 teacher and recreate the tail failure that
the method is meant to remove.

The dense block therefore changes the V208 target contract:

1. predict E-OSPA, RMSE, consensus, and byte value separately;
2. predict receiver-formation and affected-sensor tail values, rather than
   deriving safety from the aggregate score;
3. use conservative lower bounds as an admission screen;
4. rank only the admitted actions, with no-op as the default;
5. keep the strict all-tail-positive condition as a strong evidence label,
   not as the only paper-level usefulness criterion.

The same block also exposes a numerical learning hazard.  One F5 action makes
the minimum affected-sensor RMSE `1002.198%` worse.  This is a real catastrophic
receiver outcome rather than a near-zero denominator: the affected formation's
baseline sensor RMSE values range from about `8.09` to `100.60`.  Raw percent
regression would let this one tail dominate the other heads.  V208 therefore
trains on the bounded monotone target
`tanh(log(error_reference / error_candidate))`; zero and the improvement sign
are unchanged, while tables and admission thresholds remain in percent units.

## Eight-page action bank

The completed seed-211 block covers pages 72--79 and contains `831` executable
supported-label KLA actions.  Of these, `185` improve mean E-OSPA and RMSE,
`123` also improve consensus while preserving byte credit, and `73` satisfy
the zero-tolerance tail-safe label.  All five V206 teacher keys are recovered
by the truth-free shortlist.

| Page | Actions | E/R/C/bytes positive | Strict tail-safe | Best strict minimum E/R/C gain |
|--:|--:|--:|--:|--:|
| 72 | 108 | 15 | 5 | +0.932% |
| 73 | 103 | 25 | 16 | +1.078% |
| 74 | 100 | 18 | 17 | +1.286% |
| 75 | 106 | 18 | 17 | +1.149% |
| 76 | 102 | 0 | 0 | -- |
| 77 | 100 | 16 | 7 | +0.947% |
| 78 | 104 | 14 | 8 | +1.297% |
| 79 | 108 | 17 | 3 | +1.383% |

The action bank therefore contains useful signal on seven pages, but it also
contains an important counterexample: page 76 has no jointly positive
immediate action even though the V206 page-76 teacher action belongs to a
strong recursive sequence.  Immediate value is consequently a representation
and proposal target, not the final control target.

## Complete-page learnability gate

A leave-one-page-out ridge probe over five frozen random two-round graph
encoders reaches strict-action row AUC `0.7839`, top-1 strict recovery on
`3 / 8` pages, and top-5 recovery on `4 / 8` pages.  Its conservative screen
acts on two pages and both selected actions are strictly positive.  The model
is most learnable on aggregate and formation-level E-OSPA targets (Spearman
about `0.71`) and weakest on the network-worst E-OSPA/RMSE targets (`0.23` and
`0.12`).  Tail prediction, rather than aggregate ranking, is thus the limiting
representation problem.

An action-only ridge ablation reaches AUC `0.7758`, top-1 recovery on `2 / 8`
pages, and the same top-5 recovery.  This is only a small ranking gap.  The
action-only probe selects seven pages but only two selections are strictly
positive, whereas the graph ensemble selects two and both are strictly
positive.  That precision difference cannot be attributed solely to graph
context because the action-only fit is a singleton with zero ensemble
dispersion.  The defensible conclusion is narrower: graph context adds modest
ranking signal, while calibrated uncertainty or an equivalent abstention
mechanism is essential.

## Delayed-value diagnosis and decision

Three pages show why threshold tuning cannot turn the immediate probe into the
controller:

- at page 73, the V206 `F6 <- S19, [7,5]` action is nearly neutral in immediate
  E-OSPA, improves immediate RMSE by `12.047%`, and ranks only `52 / 103` by an
  aggregate immediate margin;
- at page 76, no candidate is immediately joint-positive, yet its teacher
  action contributes to the strong recursive sequence; and
- at page 78, the V206 `F3 <- S23` action is nearly neutral in immediate
  E-OSPA while improving immediate RMSE by `5.406%` and consensus by `0.872%`.

V208 therefore passes only the narrow representation gate: observable graph
features contain action-value information, but an immediate selector would
systematically miss delayed repairs and repeated locally attractive actions
can spend credit on the same formation-label pair.  V209 must use immediate
data only to pretrain a proposal model, add semantic cooldown, and learn sparse
finite-horizon returns on states induced by the evolving policy.  No recursive,
held-out, cross-seed, or cross-scene performance claim is authorized here.
