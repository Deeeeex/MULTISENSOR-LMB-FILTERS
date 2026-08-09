# V57: tracking-aligned heterogeneous joint routing

## Method decision

V56 showed that a safe source/trust change can improve tracking, but changing
one formation at a time is too weak at network scale.  Holding the best local
action for all three teacher steps raises M24 gain only from 1.313% to 1.499%.
V57 therefore changes spatial coverage before adding a learned selector.

The action is a mode vector with one entry per formation.  Each entry selects
the registered reference route or one of the current-observable source/trust
routes.  A complete vector is realized as one directed graph and one fusion
weight matrix by composing receiver rows formation by formation.  The graph,
not each local component in isolation, is the unit of execution and later
safety projection.

## First spatial-attribution screen

Before implementing a larger temporal controller, V57 measures whether joint
spatial actions contain enough tracking headroom at the already-opened M24
convoy t=60 state.

- mode count per formation: four (reference plus trust 0.30/0.50/0.70);
- candidate set: every vector within Hamming distance two of all-reference;
- M24 candidate count: 67;
- sequence: candidate at the first step, then reference for two steps;
- targets: mean tracking, minimum formation, worst sensor, consensus, attempted
  bytes, and rolling-B3 passage;
- strong gate: at least 5% mean tracking gain with all registered constraints
  nonnegative.

The candidate vectors are fixed before outcome scoring.  Truth and future
measurements are available only to the offline H=3 evaluator.  The runtime
policy receives the same sanitized current state used by V56.

## Decision after the screen

If a strong strict vector exists, the next implementation will construct a
bounded heterogeneous beam and evaluate retention, payload, physical support,
and rolling information flow on each complete proposal.  State-driven hold
and staggered recovery will then manage temporal consensus debt.

If only high-mean vectors with consensus debt exist, the same vectors become
prefixes for a two-step debt-aware recovery beam.  If no vector approaches the
5% tracking gate, V57 will stop expanding this source/trust family and redefine
the receiver/source information-value action instead.

X36 remains closed until the M24 action space clears the unchanged headroom
gate.  This is a development screen, not validation evidence.
