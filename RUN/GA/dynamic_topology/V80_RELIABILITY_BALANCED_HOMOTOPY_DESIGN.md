# V80 reliability-balanced weight homotopy design

V79 demonstrates that minimizing the recovery operator's centered norm alone
is insufficient: replacing every dominant sender changes the operator so much
that the affine forcing term dominates.  A scalar relaxation of all incoming
weights is not enough either.  On both opened anchors, every positive point on
`I + gamma (R - I)` still has centered norm above one; only the identity
operator reaches one.

V80 keeps the exact reference topology and message count.  It first computes
the effective reference matrix after deterministic current-link reliability.
Sinkhorn scaling then gives the support-preserving, relative-entropy projection
of that matrix onto the doubly stochastic set.  Rather than immediately using
the full projection, V80 evaluates the frozen homotopy

`E(alpha) = (1 - alpha) E_reference + alpha E_balanced`.

Every target effective matrix is converted back to nominal directed weights
by dividing each incoming weight by its current link reliability and
renormalizing the row.  The normal fusion routine therefore realizes the
target effective operator exactly while retaining every sender and payload.

The frozen alpha grid is
`[0, .02, .05, .10, .20, .40, .60, .80, 1.00]`.  After the same V75-safe
first-round pulse, V80 applies each point for one virtual recovery round and
measures the V77 centered arm energy relative to the parallel reference arm.
`alpha=0` exactly reproduces ordinary reference recovery.  Positive balancing
headroom exists only if an `alpha>0` strictly improves over `alpha=0`; a safe
contraction also requires the selected next-round centered energy not to
exceed the post-pulse energy.

This is a source-only response curve, not an online controller or a tracking
claim.  Its purpose is to decide whether a same-support weight correction has
useful headroom before building a multi-round predictive controller.
