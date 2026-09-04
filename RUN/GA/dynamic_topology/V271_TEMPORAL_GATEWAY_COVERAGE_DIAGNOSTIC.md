# V271 temporal gateway coverage diagnostic

## First-principles question

V242 already uses the smallest message count allowed by its local-cycle plus
bidirectional-formation-tree architecture.  Its remaining M24 weakness is a
large localization tail in one formation, not loss of instantaneous strong
connectivity.  The current route is optimized independently on every page for
link reliability and distance.  That objective can repeatedly inject
cross-formation information through the same few sensors, after which the
rest of a formation must wait for the local directed cycle to carry it.

V271 asks whether this temporal concentration is actually present before a
new controller is implemented.  It changes no posterior, fusion weight,
message, measurement, or random stream.

## Registered replay

- Policies: the exact V242 causal minimum formation backbone.
- Scenes: corrected temporal task-coupled M24 and X36 formation-braid scenes.
- Seed: 1301 for the structural replay.
- M24 tracking alignment: the already completed V248/V242 result, restricted
  to formation 4 and t=58--73.  Tracking outcomes are used only after route
  replay and never enter route construction.

For every formation and page, record:

1. which sensors directly receive and send cross-formation posteriors;
2. each sensor's age since its most recent direct cross-formation input;
3. directed local-cycle distance from a current gateway receiver to every
   member of the receiving formation; and
4. finite-horizon row-mixing of the nominal KLA matrices, measured by mean
   and maximum total-variation distance between rows of their H-step product.

The M24 event analysis also reports the association between per-sensor direct
input age and the recorded per-sensor localization RMSE.  With six sensors in
one formation this is diagnostic evidence, not a statistical claim.

## Decision rule

Proceed to a temporally balanced V272 route only if all of the following are
observed:

- at least one receiving formation in both M24 and X36 uses no more than half
  of its members as cross-formation receivers, or one member carries at least
  half of all direct cross inputs;
- the M24 tail formation has a direct-input age or local propagation-distance
  imbalance during t=58--73; and
- the finite-horizon mixing diagnostic leaves measurable contraction
  headroom, so changing the sequence of otherwise legal minimal graphs is not
  structurally vacuous.

If the gate passes, V272 must keep the V242 formation tree, KLA weights and
exact `N+2(F-1)` messages per page.  It may trade a bounded amount of current
link score for lower gateway age and better H-step mixing.  If the gate does
not pass, temporal gateway balancing is closed before any tracking run.

## Evidence boundary

V271 is a read-only route diagnostic on opened development scenes.  It can
authorize or reject one action family, but it cannot establish tracking gain,
deployment cost, X36 generalization, or a paper claim.
