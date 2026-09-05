# V275 KLA weight-balanced causal backbone

## Why this branch exists

V242 established a useful architecture: one directed local cycle per formation
plus two directed gateway messages for every edge of a causal formation tree.
It is strongly connected at every step and uses exactly
`N + 2(F - 1)` messages.  M24 development evidence also shows that it can
improve tracking while reducing communication.

Those invariants are not yet enough to invoke the standard convergence result
for an unweighted collective KLA.  In the consensus formulation of Fantacci et
al., strong connectivity supports information flow, while the consensus matrix
must also be primitive and doubly stochastic to converge to the collective KLA
with equal node weights.  The current V240/V242 policies explicitly enforce
nonnegative row-stochastic weights and supported edges, but do not enforce
column sums.

This distinction does not invalidate the observed finite-round tracking
results.  It narrows the theory claim: the current implementation is a causal,
strongly connected row-stochastic fusion process, not yet a construction with a
general guarantee of unbiased convergence to the unweighted global KLA.

Primary theorem reference:

- Fantacci et al., “Consensus Labeled Random Finite Set Filtering for
  Distributed Multi-Object Tracking,”
  <https://arxiv.org/abs/1501.01579>.

## Read-only diagnostic

`analyzeKlaWeightBalanceV275.m` replays the frozen M24 and X36 formation-braid
scenes at seed 1301.  It evaluates the matrices emitted by:

1. V240 full causal repair, with `2N` messages per step;
2. V242 minimum causal backbone, with `N + 2(F - 1)` messages per step.

For every one of the 160 steps it records the maximum row-sum deviation, maximum
column-sum deviation, message count, physical feasibility and strong
connectivity.  It also checks whether the physical adjacency and link-drop
probability schedules are symmetric.  No posterior, measurement, truth,
realized delivery or tracking score is read.

This diagnostic is intentionally separate from V272.  V272 balances how often
different sensors serve as gateways over time; it does not impose column sums
of one on each instantaneous KLA matrix, and its frozen M24 event-window screen
did not pass the tracking gate.

## Candidate construction after V274

**2026-09-05 qualification.** The construction below concerns the scheduled
matrix only. Opposite packet directions are sampled independently, and current
V240/V242 use the runtime default `missingNeighborWeightMode='renormalize'`.
Consequently, a reciprocal planned pair does not guarantee a reciprocal
delivered graph or a doubly stochastic realized matrix. Even the local
directed cycle can lose one arc. A self-weight fallback also does not guarantee
double stochasticity. V277 separates these levels; reciprocal gateways alone
are not the next method claim. The original V275 report remains a record of
scheduled matrices, not actual packet or label-wise fusion.

For an undirected formation-tree edge `(A, B)`, select one physically available
sensor pair `(a in A, b in B)` and install both directed messages:

- receiver `a` reads sender `b`;
- receiver `b` reads sender `a`.

Within each formation, assign its incident tree edges to distinct sensors.  The
local directed cycle already gives every sensor one incoming and one outgoing
dominant-weight role.  Reciprocal cross gateways then give each participating
sensor the same number of incoming and outgoing residual-weight roles.  With
the current fixed dominant/residual weights and row-completing self weight,
every column also sums to one.

The assignment must be solved jointly across both directions.  Selecting the
two directions independently can recreate V242's imbalance.  A tree-structured
dynamic program or bounded backtracking search can enforce:

- current physical feasibility of the reciprocal pair;
- a distinct endpoint sensor for every incident formation-tree edge;
- preservation of the causal incumbent tree whenever feasible;
- deterministic reliability/distance tie-breaking;
- exactly `N + 2(F - 1)` directed messages;
- nonnegative self weights and the existing one-or-two-input receiver bound.

## Decision gate

Do not implement or tune the balanced policy before the frozen V274 X36
three-arm result is known.

- If V242 has a material X36 gain, implement the reciprocal construction to
  strengthen the theory and test whether it improves the weak-formation tail.
- If full causal repair helps but V242 does not, use the balanced construction
  as a focused test of whether independent gateway directions, rather than the
  minimum message count itself, caused the loss.
- If neither dynamic arm helps, close this refinement for the current scene;
  matrix balancing alone has no evidence-backed tracking problem to solve.

After a positive development result, freeze the method before testing unopened
seeds and the existing `parallel-convoy` and `linear-relay` scene families.

## Claim boundary

The V275 diagnostic may correct the mathematical description of the current
method and authorize a targeted construction.  It cannot establish a tracking
gain, generalization, or paper contribution.  Those require paired full-episode
results and independent scene/seed validation.
