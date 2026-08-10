# V82 node-directed recovery design

V81 shows that residual soft return at the original intervention receivers is
too late: the centered existence debt has already propagated through the
high-weight within-formation star.  V82 therefore identifies the propagation
receivers from the current perturbation direction before changing any weight.

The V77 centered energy is decomposed into nonnegative per-node contributions.
For each label, candidate-minus-reference existence and normalized position
differences are centered across nodes; each node receives its `1/N` share of
the squared centered energy, and label shares are combined with the same
salience weights as V77.  The node contributions must sum back to V77 energy
within `2e-11`.

After the V75-safe pulse, V82 first applies ordinary reference recovery
virtually.  It compares each node's contribution before and after that step and
nominates at most the two receivers with largest positive growth.  This rule
uses no scale name or fixed node identity.

The one-step action bank contains:

1. ordinary reference recovery;
2. the V80 global balancing actions `G02/G05/G10`;
3. dominant-input damping by factors `0.80/0.90/0.95` on each nominated
   receiver separately and on the joint nominated set.

Dominant damping keeps the same sender and message, scales only its incoming
weight, and returns the removed weight to receiver self.  Every action is
evaluated with exact virtual mixture-aware KLA and the minimum next centered
energy is selected.  V82 is deliberately a one-step headroom screen: all four
M24/X36 historical/aligned cases must contract from the pulse before this
direction-aware action is extended to two recovery rounds.
