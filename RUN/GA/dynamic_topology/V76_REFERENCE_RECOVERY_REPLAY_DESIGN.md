# V76 reference-recovery replay design

V75 separates direct spatial conflict but does not say whether a safe local
information pulse leaves a network disagreement debt that the reference graph
can repay.  V76 tests this as a separate KLA information-flow question.

Both arms start from the same current local posteriors.  The reference arm
uses the current physical-tree route for three virtual fusion rounds.  The
candidate arm uses only the V75 direct-safe formation replacements in the
first round: M24 formation 3 or X36 formation 4.  It then uses exactly the same
reference route as the reference arm for two recovery rounds.  The historical
V71/V72 slot generation and the aligned V73 slot generation are evaluated
separately.

Every round uses the registered directed row weights and the formal
mixture-aware heavy-fusion receiver.  Current delivery reliability scales the
neighbor weights and is held fixed across the virtual rounds.  This is a
deterministic expected-weight approximation, not an exact recursive packet
outcome expectation.

After every round V76 reports two quantities:

1. the mean and upper-quartile posterior-summary gap between corresponding
   candidate-arm and reference-arm nodes;
2. each arm's own mean and upper-quartile whole-network disagreement.

A source-only recovery certificate requires both candidate--reference gap
curves to be non-increasing after the intervention.  No absolute tolerance is
tuned.  If the gap does not contract, the action is classified as
`direct-safe / recovery-unsafe` and is eligible only for a one-step pulse with
reference fallback.  If the gap contracts but the opened V72 closed-loop
outcome later deteriorates, the source-only KLA replay is insufficient to
model measurement and motion interactions; that residual must not be hidden
inside another analytic threshold.

V76 performs no prediction, target motion, measurement update, route
execution, truth scoring, future-link lookup, or model training.  It is a
mechanism diagnostic, not a tracking guarantee.
