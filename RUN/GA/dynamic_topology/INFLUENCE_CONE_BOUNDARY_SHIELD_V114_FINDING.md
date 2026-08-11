# V114 finding: whole-posterior shielding is too coarse

## Question

Can the residual F6 loss in V113 be removed by shielding the single
cross-formation boundary edge `27 -> 32` when the altered posterior first
arrives?

## Paired result

| Arm | Mean E-OSPA | Gain vs CW full | Mature min | Min formation | F6 peers | Worst sensor | Terminal consensus | Bytes | B3 |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| V113 unshielded | 78.479689 | +4.063% | +2.588% | -1.495% | -7.206% | +6.850% | +17.191% | +3.330% | pass |
| shield pages 6--8 | 78.358108 | +4.212% | +2.588% | -0.598% | -5.604% | -- | -- | +4.029% | pass |
| shield pages 5--8 | 78.319230 | +4.259% | +2.960% | -0.311% | -5.522% | +11.136% | +19.586% | +4.244% | pass |

Both candidate arms are compared with the matched clockwise full-payload
carrier.  The early arm is the V114 oracle, but no arm passes the complete
gate.

## Causal interpretation

The shield does what its local causal hypothesis predicts: receiver 32's
terminal gain becomes `+7.329%`, and network-tail and consensus metrics also
improve.  It does not solve the downstream estimation problem.  F6 sensors
33--36 still inherit an information deficit, leaving the F6 non-gateway tail
at `-5.522%`.  Starting one page earlier adds only `0.049` percentage points
of network-mean gain, so the remaining gap is not an arrival-timing error.

The failed gate therefore has a precise meaning.  The useful and harmful
parts of the message share the same posterior payload.  Dropping that payload
at the boundary removes the harmful influence, but it simultaneously removes
label information that downstream F6 sensors need.  Whole-posterior binary
abstention cannot express the required trade-off.

## Method decision

Close further boundary-timing sweeps and do not train a predictor for this
binary action space.  The next upper-bound experiment should be label-wise:

1. preserve full posterior participation for locally supported, low-debt
   labels;
2. defer only the label subset whose counterfactual influence is predicted to
   create downstream loss;
3. restore deferred labels gradually when receiving-side support becomes
   credible; and
4. compare with a reachable alternative full-payload source as a conservative
   fallback.

V114 is a retrospective oracle on X36 seed 211 at t=72.  It establishes a
mechanistic boundary, not a deployable policy or cross-scene result.
