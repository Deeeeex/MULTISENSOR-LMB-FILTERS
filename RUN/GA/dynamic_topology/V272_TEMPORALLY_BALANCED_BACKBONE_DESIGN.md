# V272 temporally balanced minimum backbone

## Research question

V242 already reaches the minimum message count within the chosen architecture:
one directed local cycle per formation and one message in each direction for
every formation-tree edge. Its remaining weakness is temporal. The
reliability-first gateway assignment repeatedly selects a small subset of
sensors, so an external posterior can enter a formation through the same node
for many pages and can require several recursive fusion steps to reach the
other nodes.

V272 asks whether the same messages and KLA weights can be embedded over time
so that external information reaches more physically eligible sensors sooner,
without replacing reliable links with arbitrarily weak ones.

## Fixed boundary

V272 preserves all of the following from V242:

- the current causal formation tree and its physical-feasibility repair;
- one current physical directed cycle inside every formation;
- exactly one directed cross-formation message per direction and formation-tree
  edge, for a total of `N + 2(F-1)` messages per page;
- dominant, cross-residual and self KLA weights;
- full-posterior payload semantics, random streams and receiver behavior.

It does not read posterior content, measurements, ground truth, future pages,
tracking outcomes or realized packet deliveries.

## Policy

For each directed formation-tree edge, the current V242 gateway is used as the
quality reference. A candidate sensor link is admitted only if its reliability
is at least 90% of the V242 link and loses no more than 0.05 in absolute
reliability. Receiver assignments remain injective within each formation.

Among admitted assignments, V272 first serves receivers that have waited the
longest for a direct cross-formation input during the preceding 12 selected
topologies. For each receiver, it reconstructs the fixed-weight product of
those past topologies and prefers a sender whose row carries more influence
from formations external to the receiver. Sender service age, current link
reliability, distance and immutable physical UIDs provide progressively later
tie breaks.

The resulting decision is causal and index-equivariant. It can be interpreted
as reliability-constrained temporal mixing: receiver debt distributes direct
injection opportunities, while the route-product term avoids rotating toward a
sender that has little useful upstream coverage.

## Structural decision gate

Before any tracking run, both corrected M24 and X36 must satisfy all of the
following on the same route replay:

1. every page is physical, strongly connected and uses the exact V242 message
   count and weights;
2. the registered link-quality floor is never violated;
3. minimum receiver coverage increases and maximum receiver concentration
   drops by at least 0.05;
4. coverage among sensors with at least one admissible opportunity increases,
   and the longest run of missed admissible opportunities decreases;
5. 12-step mean row total variation and the minimum external-formation
   influence both improve, with no regression in the averaged worst row pair.

Passing this gate authorizes only one paired M24 event-window tracking screen.
It is not itself evidence of tracking benefit or cross-scale generalization.

## Theory direction

Let `W_t` be the row-stochastic KLA weight matrix selected at page `t`. The
finite-horizon influence matrix is

`P_(t,H) = W_t W_(t-1) ... W_(t-H+1)`.

The row total-variation diameter of `P_(t,H)` measures how differently two
receivers weight the network's earlier posteriors; a smaller diameter indicates
faster weak ergodicity. The external-influence lower bound measures the least
mass any node assigns to sources outside its own formation. V272 does not yet
claim an optimal contraction bound, but its policy variables and structural
gate are defined directly on these quantities, allowing a later approximation
or regret analysis without changing the estimator.

## Evidence boundary

Structural replay is mechanism evidence only. A positive M24 event window is a
development result only. Full-episode M24, independent X36, new seeds and the
separate convoy/crossing/relay scene styles remain necessary before a paper
claim.
