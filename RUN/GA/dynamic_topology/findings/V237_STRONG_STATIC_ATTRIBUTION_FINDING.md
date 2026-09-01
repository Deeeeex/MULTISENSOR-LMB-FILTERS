# V237 strong static attribution finding

## Why the V227 gain is not yet a temporal-routing result

The V227 registered-static arm selects its route from the scenario's
pre-registered sparse base graph, whereas the dynamic arm selects from the
current physical graph.  In M24 seed 1301 the registered graph contains 58
directed edges, while the intersection of all 160 physical pages contains
all 552 possible non-self directed edges.  In X36 the corresponding counts
are 88 and 626.  The baseline therefore changes both the initial action set
and whether later reselection is allowed.

The M24 dynamic route selected at t=1 remains physical for the whole episode.
The same is true in X36 seed 1301.  Moreover, each of the five M24 incumbent
routes remains physical after its replacement.  At the harmful t=57 F4
switch, the removed and added cross-formation edges both remain physical,
both carry residual weight 0.05, and both have link reliability 0.9416.
Physical feasibility and scalar link reliability do not justify that switch.

## Required causal baseline

V237 applies the exact V227 current-physical selector at t=1, then freezes the
selected adjacency and weights.  It has the same initial candidate graph,
message count, payload mode, fusion-weight multiset, measurements, delivery
uniforms and filter seed as the dynamic arm.  Its only intervention is the
absence of later route reselection.

If V237 retains the V227 gains over the registered-static arm, those gains
belong primarily to initial route selection, not temporal adaptation.  If it
also removes the F4 tail, the next deployable method should use incumbent
preservation and switch only on physical infeasibility or a posterior-semantic
benefit certificate.  A learned switch controller is justified only after
this stronger baseline leaves measurable temporal headroom.

V237's current eligibility preflight reads future physical pages only to
confirm that the frozen arm can finish the recorded episode.  Runtime policy
selection reads no future page, target truth or tracking outcome.  A general
online method still needs a causal infeasibility fallback before any
deployment or generalization claim.
