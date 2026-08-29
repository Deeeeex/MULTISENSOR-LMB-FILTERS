# V149 M24 closure: receiver-relative edge scoring is output-misaligned

## Registered result

V149 fails the frozen M24 mechanism gate.  Its intervention-window E-OSPA
gain is `+1.639%`, below the registered `+5%` threshold.  Full-window and
mature-window gains remain positive (`+5.532%` and `+6.159%`), the minimum
sensor and formation gains are `+0.790%` and `+3.354%`, reentry is exact, and
attempted bytes decrease by `0.778%`.  Direct receiver-state access is
privileged and uncharged.  The result is therefore repository-only and the
registered V149 X36 gate remains closed.

## Decisive attribution

The failure is not caused by a sender-only score leaving the payload action
unchanged.  Relative to V148, V149 changes the aggregate working/reference
label roles from `825/212` to `185/868`; it also performs 11 shared-label
W-to-R byte-cap reversions and never violates R-label cover.  Despite this
large action change, only 7 of 1,272 final sensor--time E-OSPA cells differ
from V148, and their mean signed difference is only `+0.000384` E-OSPA
(positive means worse).  V149 changes only 9 final cells relative to the
whole-W V143 carrier.

The installed consumer explains this attenuation.  After payload fusion,
the inherited output pipeline independently compares the working and relay
labels again.  It selects the relay object for 763 of 1,124 evaluated labels
and applies the predictive whole-node readout fallback at 38 node--times.
The V149 teacher, in contrast, scores a pairwise virtual KLA before either of
these output decisions.  It therefore optimizes an intermediate posterior
that is usually not the final tracking output.

## Method decision

Do not implement a receiver-row MILP with the same V149 loss.  Jointly
optimizing a consumer-misaligned surrogate would make the combinatorial
projection exact without increasing task leverage.  A future label-role
teacher would have to evaluate the complete downstream readout and recursive
state transition, preferably with finite-horizon counterfactual outcomes;
that is a different, substantially more expensive learning problem.

The immediate cross-scale question is instead whether the simpler
whole-posterior protection-load gate has a stable delayed benefit.  V145 is
the strongest safe M24 candidate on that claim: its full/mature gains exceed
`6%`, its minimum sensor and formation gains are positive, it rejoins exactly
and does not increase communication.  One frozen X36 development audit is
therefore more informative than another label-score refinement.  It cannot
retroactively promote the below-gate M24 result and must be followed by fresh
M24/X36 validation if the delayed-horizon hypothesis survives.
