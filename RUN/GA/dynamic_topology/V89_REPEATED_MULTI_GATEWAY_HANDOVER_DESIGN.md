# V89 repeated multi-gateway handover design

## Why the action level changes again

V88 falsified receiver-level existence change as a tracking-value sign.  It
also confirmed that changing one gateway formation for one event cannot
produce a five-percent 24/36-node gain.  V89 therefore retains the V86
full-formation transport operator and changes coverage across formations and
time.

## Current-only handover nomination

At every eligible time, V89 builds the geometry-only current physical-tree
reference and inspects each cross-formation residual input.  A physical
sender from a third formation is a handover candidate when it currently has
detection-associated label support that both the receiver and incumbent
sender lack.  The reliability-weighted novel support is normalized by the
receiver formation's current existence mass.

The source gate remains `0.5%` sender novelty with at least `0.2` association
support.  Protected-support deficit is reported but is not assigned a task
sign: V88 showed that lowering a supported false label may improve tracking.
At most one gateway per receiver formation is selected, and a gateway is
eligible only if it is physically connected to every other formation member
needed by the next broadcast round.

## Evidence sequence

The first scan covers every cached V84 time from 40 to 140 in steps of four
at M24 and X36.  It reads no KLA outcome or tracking result.  The required
output is the number of simultaneous gateways, affected-formation coverage,
event frequency and protected-support exposure at both scales.

If the time axis contains repeated and multi-formation opportunities, V89
implements the online three-phase controller:

1. acquire all non-conflicting nominated gateways in parallel;
2. broadcast each acquired gateway to its complete formation;
3. use the current physical reference for one recovery round.

The cycle then repeats.  Every modified row keeps its reference message count
and positive-weight multiset, and the joint sequence must pass rolling-B3.
The full-episode paired tracking gate remains five-percent mean gain with
nonnegative worst-sensor, minimum-formation and consensus outcomes.

## Frozen executable policy

The executable candidate is
`selectRepeatedMultiGatewayHandoverV89RuntimePolicy`.  It recomputes the
current physical-tree reference on every round, stores only gateways that
survive the acquire projection, and consumes that stored set exactly once on
the following broadcast round.  A repeated call with a non-increasing time
index resets the state, so separate paired runs cannot inherit a gateway.

Acquire candidates are inserted greedily in descending current novelty.
Broadcast candidates are inserted one formation at a time.  After every
insertion the complete route must remain physical, keep exactly two messages
and the same positive weights per receiver, and keep both the sensor and
collapsed-formation graph strongly connected over the latest three selected
pages.  A rejected insertion leaves that formation on the current reference;
if all insertions are rejected, the complete round is the reference.

Direct construction at t=40 confirms that M24 executes two acquire and two
broadcast gateways with 48 messages per round, while X36 executes four and
four with 72 messages per round.  Both constructed sequences pass sensor and
formation rolling-B3.  This is a route-construction result only; the frozen
paired full-episode runner is responsible for the tracking decision.
