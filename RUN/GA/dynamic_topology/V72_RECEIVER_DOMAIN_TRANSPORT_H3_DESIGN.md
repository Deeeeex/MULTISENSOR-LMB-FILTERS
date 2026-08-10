# V72 receiver-domain transport paired H=3 design

V72 asks one narrow causal question: does the executable fixed-message V71
transport route improve tracking over the next three steps at the two frozen
M24/X36 source anchors?

Reference and candidate start from the same cached predecision posterior and
two-step selected/delivered topology history. They reuse the same current and
future measurements, link-delivery draws, and filter random seed. At the first
step, the reference executes the current-physical-tree route and the candidate
executes the already source-selected V71 route. At steps two and three both
arms recompute the current-physical-tree reference. Thus the paired difference
is attributable to one route decision rather than a hidden future selector.

The opened cases are fixed before outcome scoring:

| Scale | Seed | Anchor | Candidate formations | Receiver slots |
|:--|--:|--:|:--|--:|
| M24 merge-split | 1401 | 80 | 3 | 2 |
| X36 merge-split | 1401 | 52 | 4, 5 | 4 |

A scale passes the strong development gate only if mean E-OSPA improves by at
least 2%, the worst sensor, minimum formation, window consensus, and terminal
consensus do not degrade, selected rolling-B3 remains valid, and attempted
bytes do not increase by more than 1%. Both scales must pass independently;
one scale cannot compensate for failure in the other.

The results remain opened development evidence. They do not validate
merge-split, do not establish braided-handover generalization, and do not yet
authorize GNN training.
