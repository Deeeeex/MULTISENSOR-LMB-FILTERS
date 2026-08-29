# V154: propagation-coherent safe graph options

## Method decision

V154 keeps the complete, projector-safe graph action from V152, but changes
its temporal semantics.  A graph is no longer regenerated independently on
every filter page.  At an option boundary, the deterministic generator builds
one complete safe graph; the filter retains that graph for a formation-scale
dwell unless a physical-safety check forces fallback.

The change follows directly from the paired evidence.  V152 rank 4 creates a
large M24 tracking gain, so gateway assignment can matter.  V153 fails X36,
and its closest candidate changes the gateway map on every page and reverses
from a small mid-window gain to a tail loss.  The missing state variable is
therefore the information path already installed by the previous action, not
another edge-diversity penalty.

## Why the action is an option rather than an edge set

Let (G_k) be the complete directed KLA graph selected at an option boundary.
One cross-formation residual input enters each formation through a gateway,
and the dominant within-formation layer then transports that posterior to the
remaining members.  Replacing the gateway map on the next page changes this
transport path before its effect is represented in a graph-value target.

V154 uses a semi-Markov action

\[
    o_k = (G_k, d_F), \qquad d_F = F,
\]

where (F) is the number of formations.  M24 therefore replans every four
pages and X36 every six pages.  The rule is permutation equivariant and grows
with the formation graph rather than using an M24-tuned absolute duration.
The first eight-page pilot contains two option decisions on M24 and two on
X36, with the second X36 option contributing its first two pages.

This dwell is a causal structural scale, not a claim that KLA has fully mixed
after (F) pages.  It provides one complete formation-cycle service interval
before ordinary replanning.  Any later learned option-value model may predict
whether to stay or switch only at these boundaries.

## Frozen option bank

At each option boundary, V154 constructs the same six truth-free rank graphs
as V152 from the current observable posterior, geometry and link state.  For
rank (r), the corresponding V154 arm selects rank (r) at every boundary
and holds the returned graph between boundaries.  Static clockwise and
counter-clockwise graphs remain the paired references.

Every selected edge transmits the complete current mixture-aware LMB
posterior.  V154 does not omit labels, collapse Gaussian mixtures, cache stale
posteriors, alter nominal KLA weights, or add a second estimation payload.
The synopsis used by a future selector is control metadata only.

The graph contract remains:

- current physical reachability;
- fixed dominant weight `0.70` and residual weight `0.05`;
- row-stochastic installed KLA weights;
- exactly `2N-2F` directed transmission opportunities per page;
- one sensor-level strongly connected residual cycle, which is stronger than
  the registered rolling-B3 requirement; and
- complete-posterior byte accounting from the actual recursive trajectory.

Between option boundaries, the previous selected graph is reused only if all
of its edges remain physically reachable and its independent graph and
message-count checks still pass.  Otherwise the option terminates and the
current clockwise safe projector is executed.  No learned component may
override this fallback.

## First causal test

The first test opens only `x36-formation-fov`, seed 83, window `60:67`.  It
reuses the exact V152 continuation cache, measurements, delivery uniforms,
filter random stream and static CW/CCW shards.  Only the six V154 dwell arms
are newly executed.  This isolates temporal coherence from scene, estimator,
payload and random-number changes.

A held rank advances only if it achieves all of the following relative to the
better paired static reference:

- at least `5%` mean E-OSPA gain;
- no worst-sensor regression;
- no minimum-formation regression;
- no consensus-OSPA regression;
- no more than `5%` attempted-byte increase; and
- all physical, weight, message-count and connectivity checks pass.

If no X36 rank passes, V154 stops before M24 replication, multi-seed
expansion, richer scenes or learning.  That result would show that temporal
coherence alone cannot rescue the current gateway codebook.  If X36 passes,
the unchanged option rule is replicated on M24 seed 83 and then frozen across
the five development seeds.  M24 and X36 must each pass on at least four of
five seeds with aggregate mean gain at least `5%`; neither scale may
compensate for the other.

## Deployable value model after headroom

Only a passing cross-scale option bank authorizes learning.  The deployed
model receives the current observable posterior summaries, physical/link
graph, incumbent complete graph, incumbent age, candidate complete graph and
candidate-versus-incumbent differences.  It predicts finite-option tracking,
tail, consensus and byte value rather than imitating a retrospective rank.

At an option boundary, the exact projector first creates feasible candidates.
A calibrated joint lower-confidence rule may switch only when every safety
component is nonnegative and the configured tracking--byte utility exceeds
the incumbent.  Otherwise it stays with the current feasible option or uses
the static fallback.  This makes the learning problem graph-option value
estimation, while feasibility and dwell remain deterministic.

## Theory target

For the finite projected option set, let (V(o\mid s,G,a)) denote the
finite-dwell value vector conditioned on observable state (s), incumbent
graph (G) and graph age (a).  Let (L(o\mid s,G,a)) be a simultaneous
lower confidence bound calibrated across all candidates and registered value
components.  Selecting a switch only when (L\ge 0), otherwise staying or
falling back, gives conditional non-regression on the joint coverage event.
The violation probability is bounded by the calibration error plus measured
distribution-shift and receding-horizon mismatch.

Separately, every executed option satisfies the deterministic graph and
communication invariants above.  These are two different guarantees: the
projector certifies executable information flow, while calibration controls
empirical task-value risk.  Neither is presented as an unconditional theorem
that dynamic topology always lowers tracking error.

## Difference from closed branches

- V49 selected formation cycles using graph contraction; V154 keeps the
  estimator-aligned gateway generator and changes only temporal commitment.
- V56--V59 changed local formations or formation subsets; V154 executes a
  complete sensor graph as one indivisible action.
- V101 added dwell to posterior-admission protection; V154 never suppresses a
  posterior and instead stabilizes a full-payload graph assignment.
- V152/V153 produced edge-diverse graphs but forgot the incumbent action;
  V154 explicitly represents graph age and switching value.

## Reporting boundary

V154 is an opened-development headroom protocol.  Failed ranks and diagnostic
numbers remain repository-only.  The main progress document receives the new
method direction only after the protocol is executable, and receives numeric
results only after the registered cross-scale aggregate gate passes.
