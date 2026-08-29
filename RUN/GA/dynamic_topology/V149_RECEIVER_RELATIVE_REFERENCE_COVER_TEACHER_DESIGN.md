# V149 receiver-relative, reference-cover-preserving label-role teacher

## Question

V148's sender-only rule fails M24, and `99.929%` of its signed loss relative
to V143 occurs on the two pages that delete labels already present in the
same-edge R payload.  V149 removes that confound before asking a more useful
question: if every R label remains present, can the density source for each
shared label be chosen for the needs of a particular receiver while retaining
one payload and the exact R byte cap?

## Receiver-relative task surrogate

For a shared label, the teacher virtually fuses the receiver's current local
label state with either the sender's complete W object or its complete R
object using the registered edge weight and the installed mixture-aware
LMB-KLA.  The two fused objects are scored against all currently supported
receiver/W/R anchors.  The loss is decision aligned: it combines existence
error, capped position error, a small uncertainty term, and a large penalty
for moving a positively supported label downward through `r=0.5`.

The teacher selects W only when its virtual fused loss is strictly smaller.
An R-only label always remains R.  A W-only label is an optional extension
and is included only when it improves the same receiver-relative loss.

This is deliberately not yet an online method.  The sender does not possess
the receiver's current posterior under the registered communication contract.
V149 reads that state directly only to test action-space headroom.  It reads
no target truth, future measurement, future outcome, or paired alternative
result.  A positive result would still require a separately transmitted and
charged receiver request/synopsis before deployment.

## Direct prior-work boundary

Separated event-triggered LMB communication already compares each current
Bernoulli component with the predicted copy of the most recently broadcast
component and transmits the current component only when their KL discrepancy
crosses a threshold (Shen et al., IEEE TSP 2022,
doi:10.1109/TSP.2022.3154227).  Related event-triggered consensus LMB work
also triggers informative Bernoulli components separately (Li et al., IEEE
TAES 2023, doi:10.1109/TAES.2022.3187390).  Consequently, neither
"per-label transmission" nor "choosing current versus retained information"
is a valid novelty claim by itself.

The narrower V149 question is whether four constraints can be combined in a
useful way: the decision is conditioned on a particular receiver, its score
is computed after the installed mixture-aware KLA rather than from a
sender-only discrepancy, every R label remains covered, and the complete
one-message payload obeys a hard same-edge byte cap.  V149 evaluates these
properties edge by edge and is still only a privileged teacher.  A positive
result would motivate a receiver-row joint projector and a charged synopsis;
it would not establish priority over event-triggered component selection.

## Exact byte and support projection

The same-edge full R payload is both the label-cover baseline and byte cap.
Every R label is mandatory.  If selected W objects make the payload too
large, the lowest-value shared W choices revert to their corresponding R
objects.  Only after those reversions may W-only extensions be removed.  The
unchanged full R payload is the final fallback, so the projection cannot
introduce a label absence relative to R.

The physical route, fusion weights, delivery realization, role timing,
post-fusion readout, message count and cross-formation R-only rule remain
frozen from V143/V148.  The receiver-state access is recorded as privileged
teacher information and no communication-method claim is allowed.

## Sequential gate

M24 runs first on seed `1601` and the frozen action `25`.  X36 remains closed
unless M24 reaches at least `5%` intervention gain, nonnegative full and
mature gains, the registered sensor/formation safety thresholds, exact
rejoin, zero auxiliary payload and no attempted-byte increase.  The identical
teacher must then pass X36 before any observable synopsis or learned ranking
model is designed.

If V149 has joint headroom, the next problem is representation: transmit the
smallest receiver-need synopsis that preserves its choices, charge those
bytes, and compare an analytic scorer with a shallow model before considering
a GNN.  If the safe receiver-relative teacher has no joint headroom, the
label-role payload action family is not a useful basis for the paper and the
method must instead change route timing or physical message allocation.
