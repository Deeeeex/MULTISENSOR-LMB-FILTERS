# V79 balanced recovery operator design

V78 shows that the existing binary action space has no recovery headroom:
`CRR` is already the least amplifying schedule, while repeating the candidate
route makes M24 and X36 worse.  V79 therefore changes the recovery operator
rather than its duration.

The current reference route combines three row weights: `0.25` on the
receiver itself, `0.70` on a within-formation index-star sender, and `0.05` on
the global residual tour.  It is row-stochastic but strongly column-imbalanced
because a formation gateway supplies the dominant input to several receivers.
The resulting centered linear operator can have norm above one, so recovery
rounds are not guaranteed to contract a node-wise perturbation.

V79 retains the residual Hamiltonian tour, every receiver's weight multiset,
and the exact directed-message count.  During the two recovery rounds it
replaces only the dominant index star by a physical balanced cycle.  Before
link-reliability scaling, the self, dominant, and residual components are then
weighted permutation matrices and their sum is doubly stochastic.

Several physical cycle phases may be available.  V79 chooses a two-round phase
pair without reading posteriors or outcomes.  Let `Pi = I - 11'/N` and let
`W_q` be the expected fusion matrix after deterministic current-link
reliability scaling.  Phase pairs are ranked by

1. the smaller peak of `||Pi W_1 Pi||_2` and
   `||Pi W_2 W_1 Pi||_2`;
2. the smaller terminal bound `||Pi W_2 W_1 Pi||_2`;
3. deterministic phase indices.

The formal source-only replay keeps the V75-safe candidate pulse in round one
and compares the old `CRR` recovery with the new `CBB` recovery.  Passing V79
requires the measured centered LMB arm perturbation—not only the linear
matrix bound—to be non-increasing across all three rounds on both M24 and X36.
The common network mode remains unconstrained.  A pass would authorize a
closed-loop tracking experiment; it would not itself establish tracking gain.
