# V123: projected formation-row composition

Existing X36 actions contain a positive but incompatible heterogeneous upper
bound: different formations prefer different incoming carrier rows and
payload-participation decisions.  Their optimistic whole-window stitch reaches
`+5.293%`, but the direct page-wise graph is not rolling-B3 connected.

V123 composes receiver rows from the clockwise, V121-F1 and V121-F2 carriers.
It then applies the smallest deterministic connectivity projection found by
row fallback: F3 returns to the clockwise row on pages five and eight.  All
other selected rows remain unchanged.  Payload abstention is independently
scheduled per formation; F5 remains protected and F6 always receives full
payload.

The candidate is one indivisible eight-page sequence.  Every page must keep
60 messages, physicality, row stochasticity and the weight multiset; every
three-page sensor and formation union must be strongly connected.  Tracking
passes only with at least 5% mean gain versus clockwise full, at least 5% on
every page from page four, and nonnegative formation, terminal, worst-node,
consensus and byte metrics.

V123 is selected from opened outcomes and is an action-space upper bound, not
a deployable method or generalization result.
