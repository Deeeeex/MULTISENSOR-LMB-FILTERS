# V150 design: output-aligned label-effective KLA oracle

## Research question

The corrected communication primitive is not a reduced posterior that becomes
trivially equivalent under receiver-side moment matching.  It is an explicit
per-source, per-label decision about whether that source participates in the
current LMB-KLA fusion.  V150 asks a narrower question before any GNN is built:

> Does label-wise KLA participation contain enough recursive tracking headroom
> to produce a stable, communication-nonincreasing gain on both M24 and X36?

If this privileged upper-bound probe cannot reach the registered gate, learning
the same action space is not justified.

## Correct fusion and message semantics

- The reference and candidate both use the ordinary single-state,
  mixture-aware LMB-KLA implementation.
- A naturally absent label continues to follow the frozen missing-label rule.
- A deliberately omitted local label is named in an explicit control envelope
  and contributes zero KLA weight only for that source-label pair.
- An all-label-omitted delivery remains a real received control envelope even
  though its posterior payload is empty.
- Complete Bernoulli Gaussian mixtures are retained for every participating
  label; no receiver-side moment projection creates the result.
- The envelope costs 16 bytes plus 8 bytes for each omitted label identifier.

## Two-stage oracle

1. At an opened continuation state, construct a bounded candidate set from the
   current posterior and the frozen cross-formation residual route.  The
   observable pre-ranking uses source weight, existence disagreement, support
   disagreement and spatial disagreement.  It does not use truth or future
   measurements.
2. Replay each singleton omission through the ordinary filter and score the
   final recursive H=8 tracking outputs.
3. Rank only byte-safe positive singleton outcomes, form small prefix bundles,
   and replay those bundles independently.  This composition step uses future
   outcomes and is therefore explicitly privileged development evidence.

The intervention lasts one step; all later steps return to the registered
full-payload reference route.  This separates downstream value from a permanent
topology change.

## Registered cases and gate

| Scale | Preset | Seed | Open time | Horizon |
|:--|:--|--:|--:|--:|
| M24 | `m24-formation-fov` | 211 | 104 | 8 |
| X36 | `x36-formation-fov` | 211 | 72 | 8 |

For each scale independently, the best action must achieve at least 5% mean
E-OSPA gain while keeping worst-sensor, minimum-formation, window-consensus,
terminal-consensus and attempted-byte changes nonnegative.  Passing one scale
cannot compensate for failure on the other.

## Decision rule

- Both scales pass: retain label-effective participation as the method target;
  next derive an observable analytic score and then test a GNN approximation.
- Headroom exists but stability fails: enlarge the action constraint or add a
  safety projection before learning.
- Either scale lacks material mean headroom: stop this action space and return
  to a different fusion/routing primitive.

V150 cannot support a deployable-policy, training, validation or generalization
claim.  It only decides whether the action space deserves further investment.
