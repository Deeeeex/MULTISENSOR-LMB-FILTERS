# V48: contraction-guided residual phase scheduling

**Development decision:** retain this mechanism as a graph-only structural
probe and possible teacher, but do not promote it to the runtime method.  The
first multistyle screen finds useful M24 headroom but less than 1% median
headroom on X36, before paying any control cost.  Central plan dissemination
and per-receiver pulls are therefore stopped at the contract/cost-estimator
stage.

## Why V47 is not continued

V46 synchronized B4 reduces posterior-message attempts by 37.5%, but the first
completed M24 runtime shard shows that temporal sparsity can worsen consensus:
its mean combined consensus diagnostic is about 11% higher than the full
reference even though attempted posterior bytes fall from roughly 239.8 MB to
139.3 MB.  V47 tried to spread residual traffic according to global
posterior/ACK debt.  It is not a suitable successor because its first
same-budget M24 structural factor is worse than synchronized B4 and the global
summaries and acknowledgments required by its scheduler were not charged.

V48 changes both the optimization target and the information boundary.  It
does not rank messages using posterior contents.  It asks how to place the
same residual transmissions in time so that the KLA consensus operator loses
as little contraction as possible.

## Method

For one repaired current-graph snapshot, let \(D\) contain one dominant
incoming edge per receiver and \(R\) contain one residual incoming edge per
receiver.  Every receiver is assigned one residual-service phase
\(\phi_i\in\{1,2,3,4\}\).  In the frozen-snapshot optimization, phase \(b\)
uses

\[
A_b = D \cup \{(j\to i)\in R:\phi_i=b\}.
\]

The dominant weight is 0.70, the active residual weight is 0.20, and omitted
mass returns to the receiver's self weight.  On that frozen snapshot every
residual edge is attempted exactly once, so the candidate uses \(5N\)
posterior messages versus \(8N\) for the full reference.

The actual V46 repair is current-page and can change the sender attached to a
receiver between phases.  A deployable V48 executor must therefore carry only
the receiver phase forward and apply it to the residual input returned by the
current repair.  It may claim one residual **receiver opportunity** per window,
not service of the same physical edge.  The snapshot union and no-worse proxy
do not certify the realized time-varying four-page route.  Runtime rolling
connectivity, fallback, and the resulting message count remain unopened gates.

Let \(M_b\) be the random effective row-stochastic matrix after independent
packet delivery and the registered missing-neighbor rule, let
\(P=M_4M_3M_2M_1\), and let
\(\Pi=I-\mathbf 1\mathbf 1^\top/N\).  The existing exact recursion computes

\[
Q(\phi)=\mathbb E[P^\top\Pi P],\qquad
\rho_4(\phi)=\lambda_{\max}(\Pi Q(\phi)\Pi).
\]

V48 minimizes \(\rho_4\) over a deterministic seed bank followed by
physical-UID-ordered coordinate descent.  The bank contains all four
synchronized pulse phases, a UID-balanced schedule, a formation-staggered
schedule, and a receiver-staggered schedule.  Synchronized B4 is therefore an
explicit feasible fallback, so the selected snapshot proxy cannot be worse
than the **best of all four synchronized pulse phases**.  Phase-1 alone is not
a fair baseline because moving the same global pulse within a finite window
can change the boundary-sensitive factor.  Runtime selection also has an
independent 1% materiality gate: a non-synchronized schedule below that
threshold is replaced by the best synchronized pulse.  The headroom probe
explicitly sets the threshold to zero because its purpose is to measure the
unconstrained structural ceiling, not to authorize execution.

This is a propagation statement, not a tracking guarantee.  For exact KLA
with common positive support, complete-set-density log ratios obey the linear
mixing recursion, so \(\rho_4\) bounds their expected centered energy across
the fixed route window.  Local Bayes updates, changing label support, spatial
overlap normalizers, Gaussian-mixture approximations, and pruning enter as
unbounded disturbances and must be evaluated empirically.

## Causal and communication boundary

The optimizer reads only:

- the current repaired dominant/residual graph;
- the current link-success probability page;
- the current formation membership; and
- immutable physical sensor identifiers for deterministic tie breaking.

It reads no posterior, posterior summary, innovation statistic, delivery ACK,
measurement, target truth, future geometry, future outcome, or realized packet
draw.  During the first structural probe the current graph and reliability are
held fixed across the four-page objective.  The resulting schedule is a
causal snapshot plan, not knowledge of the next four realized route pages.

The reference and both sparse arms already require the same current route-layer
graph and link-state page.  V48's additional logical control object is a
two-bit phase assignment per registered sensor.  The control MVP uses a
32-byte wire header, 24 fixed plan-payload bytes, and
`ceil(2N/8)` packed phase bytes.  The full plan is forwarded across each edge
of a rooted sensor arborescence.  At its assigned phase, a receiver sends a
44-byte pull before the residual posterior can be attempted; this avoids a
sender transmitting on a plan that only the controller received.

Under an ideal one-hop-per-tree-edge model with no retransmissions, the plan
plus \(N\) pulls costs 2,482 bytes per M24 window and 3,859 bytes per X36
window.  Posterior summaries and global delivery ACKs are not defined and
therefore contribute exactly zero.

Those numbers are only an optimistic incremental lower bound.  The prototype
does not construct the tree, bind schedule delivery to executed topology, or
model control loss, retransmissions, latency, shared route discovery, or
link/network headers.  In a lossy tree, single-shot all-node agreement becomes
less likely as the network grows.  Consequently the current artifact forbids
incremental-cost, same-total-byte, and end-to-end network-saving claims.  A
later runtime protocol must construct and validate the tree, execute a
missing-schedule fallback, and charge every control attempt before any of
those claims open.

## Multistyle falsification result

On the seed-41 graph snapshots, without reading posterior or truth:

| Scale | Style | Best synchronized | Best after three sweeps | Relative reduction |
|:--|:--|--:|--:|--:|
| M24 | radial | 0.899619 | 0.888833 | 1.20% |
| M24 | convoy | 0.972606 | 0.956956 | 1.61% |
| M24 | relay | 0.973748 | 0.957177 | 1.70% |
| M24 | crossing | 0.972473 | 0.960537 | 1.23% |
| X36 | radial | 0.952841 | 0.951349 | 0.16% |
| X36 | convoy | 0.988575 | 0.980396 | 0.83% |
| X36 | relay | 0.990002 | 0.982181 | 0.79% |
| X36 | crossing | 0.989502 | 0.982932 | 0.66% |

M24 has a 1.42% median reduction, while X36 has only about 0.73%.  The radial
lagged-window replay then uses the completed \(t-1\) snapshot to schedule the
actual \(t:t+3\) pages.  It obtains 1.20% on M24 and 0.16% on X36 against both
the causal synchronized comparator and the posthoc best synchronized pulse;
the repaired route does not change inside either tested window.  The weak X36
result is therefore not explained by a frozen-page mismatch in these cases.

The larger apparent gains previously measured against phase 1 were almost
entirely pulse-placement effects.  Even the unconstrained graph-only ceiling
is below the preregistered 1% materiality gate on every seed-41 X36 style, and
that ceiling excludes nonzero control traffic.  The all-receiver phase vector
is rejected as the primary runtime mechanism.  No tracking or communication-
saving claim is opened by these development results.

## Successor direction

The next structural screen keeps synchronized B4 as the default and changes
only a small subset of cross-formation residual traffic.  At a residual burst,
all local residuals and one mandatory directed formation cycle remain in
place.  Only edges on the opposite cross-formation cycle may be delayed by one
step, and the action must preserve the exact four-step residual-opportunity
budget.  This is intended to retain the strong synchronized mixing event while
testing the protection-and-staggered-recovery timing signal observed in V35
and V37.  The first screen is graph-only and enumerative; posterior-based local
bids and their charged commit protocol remain closed until both M24 and X36
show material structural headroom.

## Development gates

1. **Graph-only headroom.** The first four-style, one-seed screen is complete
   and rejects promotion because X36 median headroom is below 1%.  A frozen
   rerun may archive the result, but a full32 expansion is not required before
   stopping this mechanism.
2. **Permutation and tamper tests.** Reorder sensors and formations, recompute
   the physical schedule, and require exact identity.  Independently replay
   the optimizer rather than accepting self-reported scores or hashes.
3. **Time-varying structural replay.** Use the completed \(t-1\) snapshot to
   plan the next four-step window, then carry receiver phases through the real
   current-page V46 repair for complete 160-step routes.  Compare realized
   rolling factors, route churn, computation time and fallback rate against
   full V46 and synchronized B4.  Do not reuse the snapshot edge-union claim.
4. **Control protocol and short paired filter test.** Closed for this method.
   The present files define only the information boundary and optimistic cost
   lower bound so the rejected alternative remains auditable.
5. **Tracking development.** Not authorized for this method.
6. **Scale confirmation.** Not authorized for this method.  The successor
   ring-preserving defer screen must independently reopen these gates.

## Data-driven extension

The exact optimizer can later generate teacher schedules for a permutation-
equivariant graph model.  A learned model may propose a phase vector to reduce
runtime, but the executed action must still pass the exact budget, physical
support, rolling-connectivity, and no-worse-than-synchronized contraction
checks.  The GNN is therefore an optional acceleration layer, not the source
of safety or the first evidence claim.
