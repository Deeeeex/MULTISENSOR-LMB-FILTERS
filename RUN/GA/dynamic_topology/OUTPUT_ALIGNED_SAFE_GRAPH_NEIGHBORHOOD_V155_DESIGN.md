# V155: output-aligned safe graph-neighborhood oracle

## Decision question

V152--V154 reject the six score-generated graph ranks on X36, but they do not
separate two explanations: either complete full-posterior routing has no useful
headroom, or the six proposals simply miss the useful part of the feasible
graph space.  V155 tests proposal coverage directly before another analytic
score, dwell rule or GNN is designed.

The experiment is an offline action-space oracle.  Candidate graphs are built
without truth or future outcomes; the paired recursive tracking outcomes are
used only after execution to select the best complete graph.  A learned model
is forbidden until this stronger graph neighborhood contains a material,
tail-safe X36 action.

## Frozen local graph neighborhood

The center of the neighborhood is the better V152 X36 static reference: the
clockwise backbone--residual spliced cycle with dominant weight `0.70` and
residual weight `0.05`.  Its high-weight within-formation backbone remains
unchanged.

One candidate changes the residual-cycle cut in exactly one formation.  The
sender released by that cut is reconnected through the same clockwise
formation order, so changing one cut changes the two adjacent cross-formation
sensor edges while preserving the complete residual permutation.  With six
sensors per formation, two residual inputs coincide with dominant inputs and
cannot be cut without changing the message count.  Four nonduplicate cuts
remain; one is the reference and three are alternatives.  X36 therefore has
exactly

\[
    6\,(4-1)=18
\]

nonreference radius-one candidates.  The structural preflight must reproduce
that count; otherwise tracking is not authorized.

Candidates are ordered canonically by changed rank-space formation position
and receiver index.  Numeric formation identity, posterior score, target truth
and future outcome do not affect enumeration.  Each selected graph is built at
the first continuation page and held for all eight pages.  If any selected edge
becomes physically unavailable, the arm falls back to the clockwise reference
and is ineligible for the headroom gate.

Every arm preserves:

- the complete mixture-aware LMB posterior on every selected edge;
- the nominal `0.70 / 0.05` KLA weights and row stochasticity;
- exactly `2N-2F = 60` directed transmission opportunities per page;
- the fixed dominant within-formation backbone;
- a one-step strongly connected residual sensor cycle and formation ring; and
- the paired measurements, delivery uniforms, filter random stream and
  continuation state used by V152--V154.

This is not the closed V27 multi-gateway action.  V27 replaced several local
residual edges with two or three cross-formation lanes and showed that faster
global mixing can amplify KLA conflict.  V155 retains exactly one gateway per
formation and asks whether the *identity* of one gateway pair has useful
output-aligned leverage.  It also differs from V49, which ranked formation
cycles by graph contraction rather than recursive tracking value.

## Registered Stage-A gate

The first and only opened case is `x36-formation-fov`, seed `83`, continuation
window `60:67`.  The saved V152 clockwise and counter-clockwise shards remain
the paired static controls; only the 18 radius-one arms are newly executed.

Relative to the better static control, a candidate is admissible only if:

- mean E-OSPA improves;
- worst-sensor E-OSPA does not regress;
- the weakest formation does not regress;
- consensus OSPA does not regress;
- attempted bytes increase by at most `5%`;
- the requested graph executes on all eight pages without fallback; and
- all message-count and strong-connectivity checks pass.

Stage A passes the material headroom gate only when an admissible candidate
improves mean E-OSPA by at least `5%`.

## Bounded interaction follow-up

If Stage A has at least one admissible candidate and the best admissible mean
gain is positive but below `5%`, one radius-two follow-up is allowed.  It may
combine only the eight best admissible radius-one changes, ranked by mean
E-OSPA, and evaluates every structurally compatible pair (at most 28 arms).
The pair graph is constructed directly from the two cut changes and replayed
independently; singleton outcome effects are not added algebraically.

If Stage A has no admissible positive action, the radius-two follow-up is not
run.  Together with the V27 multi-lane and V49 contraction failures, that
result closes the current fixed-weight, full-posterior graph-only search.  It
does not prove that every possible communication or fusion policy is useless.

## Learning boundary

Only an X36 candidate clearing the `5%` Stage-A or radius-two gate authorizes
M24 replication and then a multi-seed graph-value dataset.  The deployed model
would receive current observable posterior summaries, link state, incumbent
graph and a complete candidate-versus-reference graph difference.  It predicts
finite-horizon tracking, tail, consensus and byte value; a deterministic
projector continues to enforce graph feasibility and static fallback.

The first learned comparator is a shallow permutation-equivariant set/edge
model.  A temporal GNN is justified only if it improves whole-seed candidate
ranking and safe-action recall across M24 and X36.  Learning never emits an
unprojected graph and never substitutes for action-space headroom.

## Reporting boundary

V155 is opened-development evidence.  Failed arms and all numeric diagnostics
remain repository-only.  The main progress document receives a numeric result
only after the unchanged rule passes both scales and the registered multi-seed
gate; until then it may record only the stable decision to test graph-proposal
coverage with an output-aligned safe neighborhood.
