# V190 recursive teacher closure: positive label repair is necessary but not sufficient

## Result

The opened H=3 recursive screens close the single-positive-label branch.  A
mixture-aware label-wise KLA update can repair the known X36 F2 deficit, but no
tested positive single-label update repairs the remaining X36 F5 or M24 F4
failure.  The best current full-window method therefore remains V187; these
teacher screens are mechanism evidence, not a new deployable candidate.

| Scale / forced action | E-OSPA gain | RMSE gain | Consensus gain | Byte saving | Binding local result |
|:--|--:|--:|--:|--:|:--|
| X36 F2, source 19 label `[13 12]`, hard replacement | +3.641% | +6.098% | +7.164% | +5.161% | F2 RMSE +62.290%; F1/F3/F5 remain negative |
| X36 F2, source 19 label `[13 12]`, label KLA, source weight 0.5 | +3.647% | +6.211% | +7.406% | +5.211% | F2 RMSE +63.564%; F1/F3/F5 remain negative |
| X36 F5, online top proposal, hard replacement | +2.671% | -1.406% | +3.973% | +5.908% | F5 RMSE -14.691% |
| X36 F5, source 2 label `[13 11]`, hard replacement | +2.799% | -0.719% | +6.134% | +5.997% | F5 RMSE -1.978%; inherited F2 RMSE -14.198% remains |
| M24 F2, label KLA, source weight 0.5 | +5.187% | +0.641% | +12.959% | +4.431% | Worse than the V99 no-op on every mean objective |
| M24 F4, source 10 label `[25 15]`, hard replacement | +7.993% | +2.002% | +19.410% | +4.040% | F4 RMSE worsens from -126.599% to -158.439% |

The matched no-repair V99 arms are important context.  X36 V99 gives
`+2.802%` E-OSPA, `-0.666%` RMSE, `+5.149%` consensus and `+6.550%` byte
saving.  M24 V99 gives `+9.044%`, `+3.734%`, `+21.104%` and `+5.080%`,
respectively.  Consequently, an action is useful only if it improves the
recursive V99 state, not merely if it scores above the static reference in an
offline one-step proxy.

## Method decision

The next controller must expose at least three actions per formation and page:

1. keep the V99 omission unchanged;
2. release the formation from omission and restore the ordinary full posterior;
3. apply a sparse positive complete-label update, preferably by label-wise KLA
   when both receiver and source carry usable evidence.

The M24 F4 failure is dominated by sensor 20 at t=104.  V99 changes its MAP
cardinality from 9 to 12 and admits a stale, high-covariance label `[1 3]`,
which produces a large position-RMSE spike while still improving E-OSPA.  This
is a payload-withholding/extraction-risk failure, not an absent positive-label
failure.  The immediate next teacher must therefore remove F4 from V99's
withheld-formation set at t=104 and charge the naturally restored full payload.
If that restores F4 RMSE while preserving positive mean gains and byte saving,
the deployable gate should be based on causal extraction-risk signals such as
MAP-cardinality margin, low existence, high covariance and inter-node
disagreement.  Truth-labelled one-step update strength must not be used as an
online action label.

## Evidence boundary

All actions above are forced first-page teacher interventions with ideal repair
delivery charged but not sampled.  They use the same cached measurements,
links, RNG state and H=3 continuation as their paired static and V99 arms.
They decide which action families remain worth implementing; they do not
support cross-seed, cross-scale or paper-level performance claims.
