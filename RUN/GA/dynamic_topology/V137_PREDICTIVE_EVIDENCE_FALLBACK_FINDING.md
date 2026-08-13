# V137 finding: pre-fusion scoring is safer but causally late

V137 is a repository-only failed mechanism screen.  It misses the frozen M24
intervention gate, so X36 was not run and the result must not be copied into
the main progress document.

## Frozen M24 result

- Intervention E-OSPA gain: **+4.474%** (required: at least +5%).
- Full-window / mature-window gain: **+0.407% / +0.000%**.
- Minimum sensor / formation gain: **-0.001% / +0.000%**.
- Exact whole-formation reentry match: **100%**.
- Charged attempted-byte delta: **+88.727%**.
- The causal W/R predictive-score rule selected R only six times.

Relative to V136, the rule almost removes the worst-node regression, but it
also reduces intervention gain from `+5.800%` to `+4.474%`.  It therefore
improves safety without preserving the registered material-gain target.

## Mechanism interpretation

Five of the six fallbacks occur on page 5 in formation 2, where sensors 9--12
are genuinely harmed by the propagated W state.  However, their first large
loss already occurs on page 4 while their local W and R predictions are still
identical and the W-minus-R predictive-score margins are exactly zero.  The
harm is introduced by the **current intra-formation fusion**, after the
pre-fusion score has already made its decision.  A node-level forecast gate is
therefore causally one stage too early for this failure mode.

The remaining fallback, sensor 19 on page 3, suppresses useful formation-4 W
propagation.  Because V137 replaces the estimator state before the next page,
one conservative local decision contaminates later W messages and removes
gain from peers that did not fail their own evidence checks.

## Method decision

Do not tune the predictive-score margin.  The next bounded mechanism must act
after the risky intra-formation W fusion and must not alter the hidden W state
used for future propagation.  At a protected receiver, use current local
per-label observation evidence to decide whether a W-fused label or the exact
R-fused label better preserves the receiver's local posterior.  Apply this
choice only to the current extracted output; let W continue as a shadow state,
and retain exact whole-formation R reentry.  This moves the safety decision to
the causal point where the error is created and prevents a single fallback
from erasing useful network propagation.
