# V242 causal minimum formation backbone

## Research question

V240 answers when a physically infeasible formation tree should be repaired,
but it still transmits two posterior inputs to every sensor. Consequently it
can isolate the value of topology repair but cannot by itself deliver the
requested communication reduction. V242 asks whether the same causal tree
repair can preserve instantaneous information flow with the smallest payload
count under an explicit formation architecture.

## Construction

For N sensors partitioned into F formations:

1. keep one current physical directed cycle inside every formation;
2. select a current physical formation spanning tree using V240;
3. for every undirected tree edge, retain one gateway message in each
   direction;
4. omit the second local-cycle residual inputs and return their weight to the
   corresponding receiver self posterior.

The resulting message count is

N + 2(F - 1),

instead of the V240 reference count 2N. With the current scene family this
means 30 instead of 48 messages for M24, 46 instead of 72 for X36, and 62
instead of 96 for X48.

## Structural statement

Each local directed cycle is strongly connected. Consider any source sensor
and destination sensor. The source can reach the outgoing gateway of its
formation through the local cycle. The unique path between source and
destination formations in the formation tree can then be traversed because
each tree edge has one gateway in both directions. The destination gateway
can reach the destination sensor through its local cycle. Therefore the
union graph is strongly connected.

The count N + 2(F - 1) is minimal only under the stated architecture: every
formation must retain a directed local cycle, costing exactly N messages in
total, and every edge of an undirected formation tree must be realized in
both directions, costing 2(F - 1). V242 does not claim a global lower bound
over arbitrary directed sensor graphs.

## Fusion weights

Receivers with a cross-formation gateway retain the V240 weights
(self, dominant, cross) = (0.25, 0.70, 0.05). Receivers without a cross
gateway omit the local residual input and return its mass to self, yielding
(self, dominant) = (0.30, 0.70). Every row remains a convex KLA weight vector
and positive weight support exactly matches the executed route.

## Evidence boundary and next decision

The opened seed 1301 is used only for structural development. The first gate
checks exact message count, current physical feasibility, instantaneous
strong connectivity, causal tree replacement, and fusion-weight validity on
M24, X36, and X48. No tracking claim is allowed at this stage.

The V241 two-input full episode remains the mechanism check: if restoring the
formation tree does not improve tracking or consistency, sparse V242 tracking
is not justified. If V241 is positive, V242 becomes the communication arm and
is compared against both the fixed-tree baseline and the 2N V240 reference.
The full-episode runner therefore accepts a completed V241 result, verifies
that reference gate, reuses both recorded arms, and executes only the new V242
arm. This keeps the comparison deterministic without repeating two long
baselines.
