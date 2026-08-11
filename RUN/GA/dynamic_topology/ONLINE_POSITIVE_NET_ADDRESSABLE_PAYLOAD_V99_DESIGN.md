# V99 online positive-net payload participation

## Method decision

V97 chooses every currently safe formation whose observable rescue exceeds
its useful loss, but freezes that set for the full three-step window.  V98
shows that this set changes after recursive fusion.  V99 therefore retains
the V97 spatial rule and changes only its temporal semantics: after every
local update, it recomputes the current safe positive-net formation set and
uses that set for the same step's fusion payload plan.

The physical carrier graph, directed message opportunities and fusion weights
remain the fixed counter-clockwise static reference.  A selected receiver
formation still receives the compact control synopsis but does not consume
complete cross-formation posterior payload on the selected carrier edges.
The selector uses the current pre-fusion posterior, current geometry and past
selected graph only.  It does not use target truth, future measurements or
future tracking outcomes.

## Falsifiable comparison

The first screen compares three paired arms at the existing X36 t=72 opened
state:

1. static reference with full posterior participation;
2. V97 with the initial positive-net set frozen for all three steps;
3. V99 with the same initial set and online re-selection at later steps.

All arms share the cached posterior, measurements, link uniforms, filter RNG,
carrier graph, fusion weights, horizon and communication model.  V99 advances
only if its mean E-OSPA gain over the static reference reaches 5%, no sensor or
formation tail regresses, consensus does not regress, the rolling B3 checks
pass, and attempted communication does not increase over the static arm.

If X36 t=72 passes, the same frozen implementation is run on both M24 anchors
and X36 t=100.  Scene expansion is deferred until all four matched-static
anchors pass; otherwise the result identifies the next missing action rather
than being hidden by broader scenario averaging.
