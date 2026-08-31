# V194: set-safe projection fixes RMSE but over-releases recursively

## Paired M24/X36 results

V194 recomputes the V99 omission proposal and removes every formation whose
candidate marginal LMB extraction contains a label without current receiver
measurement support.

| Scale | E-OSPA gain | RMSE gain | Consensus gain | Byte saving | Minimum formation RMSE gain |
|:--|--:|--:|--:|--:|--:|
| M24 | +2.543% | +10.713% | +5.650% | +0.272% | 0% |
| X36 | +0.946% | +0.779% | +1.978% | +3.448% | -0.069% |

The first-page projection matches the V193 analysis exactly:

- M24 proposal F1+F3+F4 becomes F1+F3; F4 is released;
- X36 proposal F1+F2+F4+F5 becomes F1+F4; F2 and F5 are released.

This closes the large targeted RMSE gaps.  In X36, F2 and F3 return to 0%,
F5 reaches +0.004%, and only the already-negligible F1 gap remains at
-0.069%.  Both scales retain positive network-level E-OSPA, RMSE, consensus
and communication gains.

## Recursive failure mode

The current-frame rule is too conservative after the first update:

| Scale | Page 1 releases | Page 2 releases | Page 3 releases |
|:--|:--|:--|:--|
| M24 | F4 | F1+F2 | F1+F2 |
| X36 | F2+F5 | F3+F5 | F3 |

A true established track can have zero detection-association mass on one page
because of a missed detection.  V194 then treats its entry into the candidate
MAP set as unsupported even when the label was recently supported or is
currently supported by a reachable peer.  Repeated full-posterior fallback
removes much of the useful omission action: relative to the one-page M24 F4
teacher, V194 reduces E-OSPA gain from +7.521% to +2.543% and byte saving from
+3.562% to +0.272%.  The same pattern limits X36 E-OSPA gain to +0.946%.

## Method decision

V194 establishes the correct action hierarchy and a conservative safety
fallback, but the support definition must become temporal/network-aware.  The
next analysis should evaluate an entering label as supported when either:

1. the receiver has current measurement association;
2. an active reachable peer has current association for the same label; or
3. the receiver had sufficiently recent support and the predicted track
   remains dynamically coherent.

The first candidate is current network support because it is already
available in the V99 counterfactual and adds no hidden state.  Short-term
receiver support memory is added only if network support does not preserve the
desired first-page releases while suppressing the later false releases.

## Evidence boundary

V194 uses no truth, future measurement/outcome, time identifier, formation
identifier or numeric label feature.  These paired seed-211 H=3 results are
opened method-development evidence and do not refresh the current full-window
best method V187.

