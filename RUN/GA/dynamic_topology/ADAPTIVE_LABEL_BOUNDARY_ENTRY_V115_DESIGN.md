# V115: adaptive label-wise boundary entry

## Decision being tested

V114 established that whole-posterior shielding on `27 -> 32` protects the
F6 entry receiver but deprives downstream F6 peers of useful information.
V115 changes the granularity of this decision without changing the physical
route.  The V113 clockwise carrier and its F2--F5 whole-source abstention
schedule remain fixed.  On pages 5--8, only the F6 boundary message is split
by label: every admitted label carries its complete Bernoulli Gaussian
mixture, while every omitted label explicitly abstains from the KLA update.
An omitted label is not treated as low-existence evidence.

## Minimal candidate family

All rules are recomputed from the current sender and receiver posterior and
use no target truth or future measurements.

| Arm | Label admitted on 27 -> 32 | Interpretation |
|:--|:--|:--|
| supported | sender association support at least 0.20 | conservative positive evidence |
| supported or high-r | supported, or sender existence at least 0.50 | preserve mature sender tracks |
| receiver need | supported, or receiver support below 0.20 | admit labels the receiver may be missing |

The rules are intentionally semantic rather than a dense threshold sweep.
They test whether label granularity itself contains enough headroom and
whether useful re-entry should be driven by sender confidence or receiver
need.

## Invariants and gate

The physical graph, clockwise fusion weights, attempted link opportunities,
delivery uniforms, filter RNG and rolling B3 route are paired.  F2--F5 keep
the V113 whole-source abstention action.  Only F6 uses label whitelisting.
Communication accounting includes the complete selected-label mixtures and
the same control synopsis.

The primary comparison is the matched clockwise full-payload carrier.  A
candidate must achieve at least five-percent mean and mature-page gain, with
nonnegative worst-sensor, minimum-formation, F6 non-gateway terminal,
consensus and communication changes, while preserving B3.  V113 unshielded
and V114 early whole-posterior shield are retained as mechanism references.

If none of the three semantic rules passes, the next useful upper bound is a
privileged per-label delayed-return oracle or a physically reachable
alternative full-payload source.  Further support/existence threshold sweeps
are not justified by a failed action-family gate.  If one rule passes, its
observable features can seed the time-expanded risk predictor and safety
projector before any new scene is opened.

V115 uses an already-opened X36 anchor and an action family chosen after V114.
It is method-development evidence, not validation or generalization.
