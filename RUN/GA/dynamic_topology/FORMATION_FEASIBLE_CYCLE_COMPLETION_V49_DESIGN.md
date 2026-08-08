# V49: causal feasible-cycle completion

## Motivation

V46 repairs the registered formation backbone with the fewest current-page
edits needed to remain connected.  This is robust to broken registered links,
but connectivity alone is a weak objective.  In convoy, relay, and crossing
scenes the registered backbone is an initial-geometry MST, so the repaired
formation graph normally remains a bidirectional tree.  Synchronized B4 then
has only one global mixing path and its exact propagation factor becomes
noticeably worse as the number of formations grows.

V49 changes the route projection objective, not the posterior payload.  When
the current physical formation graph supports a cycle, it replaces the tree
residual backbone with a physically executable bidirectional formation cycle.
The dominant local layer is kept bitwise unchanged.  If no cycle is feasible,
the exact score is nonfinite, the work cap is exceeded, the improvement is
below 1%, or any invariant fails, V49 returns the input V46 route bit for bit.

## Route construction

The current physical sensor graph is collapsed to a formation graph.  A
formation pair is usable only when current sensor links support both
directions.  Candidate Hamiltonian cycles are canonicalized by:

- fixing the smallest physical formation UID as the root;
- visiting every formation exactly once;
- requiring the second UID to be smaller than the final UID, which removes
  reverse duplicates; and
- using physical UID order for every remaining tie.

Quotient-graph feasibility is not sufficient.  Each proposed cycle is passed
through the existing V43 route constructor, which must assign the two incoming
cross arcs of every formation to distinct receivers and maintain one distinct
dominant and residual input per sensor.

Two proposal modes are implemented:

1. `reliability-proposal-topk` ranks all physical cycles by their worst and
   mean current directed link reliability, materializes at most three feasible
   proposals, and applies the exact certificate to those proposals plus V46.
   This is the default executable development mode.
2. `exact-contraction-enumeration` scores every feasible cycle.  It is an
   exponential development oracle and is not presented as a scalable runtime
   algorithm.

The canonical enumeration has a hard work cap of 10,000 candidate orders.  A
larger formation problem falls back to V46; a learned or combinatorial
proposer may later replace enumeration without replacing the exact gate.

## Exact gate and theoretical boundary

For a proposed route, let the synchronized residual burst be phase 1 of a
four-step window.  The dominant layer is present at every phase with weight
0.70; all residual inputs are present at the burst with weight 0.20; omitted
mass returns to self.  With independent delivery and the registered
missing-neighbor rule, the existing recursion computes

\[
Q=\mathbb E[P^\top\Pi P],\qquad
\rho_4=\lambda_{\max}(\Pi Q\Pi).
\]

V49 executes a proposed cycle only when its frozen-current-page \(\rho_4\) is
at least 1% below the exact V46 value.  Ties and smaller improvements select
V46.  Both arms use the same fixed phase-1 pulse, reliability page, weights,
missing-neighbor rule, and certificate implementation.  A best-of-four pulse
mode exists only as an explicitly marked development oracle.

This certificate describes expected centered-L2 propagation for the frozen
route/reliability model.  It is not a standalone tracking objective and does
not bound local Bayes updates, label loss, spatial-overlap normalization,
Gaussian-mixture approximation, or time-varying execution.  No tracking-
safety or time-varying no-worse claim is currently allowed.

## Communication composition

Both V46 and V49 references use exactly \(2N\) directed messages per full
step: one dominant and one residual input per receiver.  Under synchronized
B4, both attempt \(5N\) posterior messages per four steps, versus \(8N\) for
the full reference.

Equal counts do not mean identical composition.  A bidirectional formation
tree uses \(2(F-1)\) cross residuals; a bidirectional cycle uses \(2F\).
Cycle completion therefore replaces two local residual messages with two
cross-formation residual messages while keeping the total count fixed.  Since
posterior byte sizes can differ and route discovery/commit is not implemented,
same-total-byte and end-to-end saving claims remain closed.

## Attribution

The method and experiments keep three interventions separate:

- A: V46 route + synchronized B4;
- B: feasible-cycle-completed route + synchronized B4;
- C: cycle route + one-step defer ablation.

The first focus-page study shows that nearly all improvement is A→B.  B→C is
only 0%–0.22%, so one-step defer is not part of the primary method and is not
allowed to inherit the route gain.

## Current development evidence

The exact-enumeration, best-of-four exploratory screen found approximately
6.4%–7.8% route gain on non-radial M24 and 3.5%–4.1% on non-radial X36.  The
radial gains were only 0.19%–0.31% and therefore fall below the runtime 1%
gate.  These values motivated the method pivot but are not executable-runtime
evidence because that screen optimized pulse placement.

A separate cheap availability audit covers four seeds, eight styles, and all
160 pages per case.  All 5,120 physical pages contain at least one formation
cycle.  The UID-first cycle is unchanged in the static convoy/relay cases;
the largest observed change count is seven over 159 transitions in X36
crossing.  This proves action availability for the registered scenarios, not
certificate or tracking success.

The frozen runtime-compatible selector was then replayed at all 320 absolute
B4 boundaries in the eight seed-41 scenarios.  It selected a cycle in 271
windows; all 271 selected windows improved the posthoc four-page factor and no
window was worse than V46.  Every non-radial window was selected.  M24
non-radial mean improvements are 6.064%--8.750%; X36 non-radial means are
3.454%--3.846%, with a 2.755% worst selected X36 improvement.  These are
single-seed, correlated-window structural results, not tracking,
communication-byte, or online-safety results.

The planning and posthoc factors happen to match in this matrix because the
active route/reliability state relevant to each audited product remains
unchanged inside its B4 window.  This observed equality is not a method
invariant.  X36 crossing also changes its selected formation-pair set 16 times
over 39 transitions, so control-plane churn remains a first-order concern.

The exact-enumeration focus oracle agrees with top-3 on M24 but exposes
0.377--1.366 percentage points of extra improvement on X36 and selects an X36
radial cycle that top-3 misses.  Dense X36 pages require up to 61 certificate
evaluations and about 15 seconds on the development machine.  The default
selector therefore certifies only the proposals it evaluates; it does not
claim global or near-global optimality.

## Next gates

1. Run a non-scoring real-filter smoke against V46 synchronized B4 with the
   same physical-UID-keyed delivery draws and exact `[2N,N,N,N]` message
   counts.
2. Independently replay every installed V49 route and weight page, and test
   that changing posterior contents cannot change the route.
3. Measure correlated-loss sensitivity and separate posterior payload bytes
   from route-discovery/commit bytes.
4. Add an atomic route-hash dissemination/fallback protocol before any
   distributed-execution claim.
5. Freeze the proposal rule after either improving the X36 oracle gap or
   explicitly accepting top-3 as a bounded-work heuristic.
6. Only then open multi-seed episode-level tracking development; individual
   B4 windows are not independent statistical samples.

## Data-driven extension

A permutation-equivariant graph model may propose a small top-k set of cycles
or predict whether cycle completion is worth attempting.  The observed X36
oracle gap gives this proposal-learning problem a concrete target.  It may
read only the registered causal input boundary.  Every proposal must still
pass the distinct-receiver route constructor, physical support, fixed budget,
exact certificate, 1% materiality gate, and global V46 fallback.  Learning is
an acceleration/proposal layer, not the source of the safety claim.
