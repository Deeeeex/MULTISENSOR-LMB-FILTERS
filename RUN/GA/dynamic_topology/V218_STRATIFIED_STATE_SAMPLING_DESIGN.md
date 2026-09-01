# V218 stratified direct-graph state sampling

## Method decision

V214 selected two separated local maxima of receiver `need_max` from each
base-route trajectory.  The resulting M24 H=3 labels show that this is not a
sufficient acquisition rule.  Receiver need describes where information may
be missing, but the value of withholding a complete incoming posterior also
depends on whether that posterior is redundant, conflicts with a better local
state, or creates delayed Gaussian-mixture payload growth.

V218 changes offline state acquisition, not the online action rule.  It runs
the same current-physical V214 full-payload reference route and reads the V215
formation features that are already computed on each page.  After the complete
truth-free trajectory is available, it selects one formation-time pair from
each of four causal strata.  Selected states later receive exact paired H=3
labels for no-op, one-page withholding and withholding plus one supported
complete-label KLA action.

## Frozen causal strata

Every component is converted to a midrank percentile over all
formation-time pairs in the same complete trajectory.  A `+` direction uses
the percentile directly; a `-` direction uses one minus the percentile.  The
stratum score is the arithmetic mean of its directed components.

| Stratum | Observable components | Direction | Intended label coverage |
|:--|:--|:--:|:--|
| Redundancy saver | incoming payload share; active-label overlap; positive existence gain; positive precision gain; source quality advantage; state discrepancy | `+ + - - - -` | High-cost incoming posterior with little marginal information; withholding may save bytes without changing output |
| Conflict relief | state discrepancy; negative existence gap; negative precision gap; source quality advantage | `+ + + -` | Incoming source disagrees with, and is not better than, the receiver; withholding may improve tracking or consistency |
| External-information dependency | receiver need; positive existence gain; positive precision gain; source quality advantage | `+ + + +` | Receiver depends on genuinely better external information; no-op or a supported complete-label repair should dominate blind withholding |
| Payload-rebound pressure | incoming payload share; incoming bytes per edge; receiver components per label; sender components per label | `+ + + +` | High mixture complexity may make a current saving reappear on later full-payload pages |

These labels describe hypotheses to sample, not assumptions about their
outcomes.  In particular, a high redundancy score does not certify harmless
withholding, and a high dependency score does not force a repair.

## Deterministic selection

The selector evaluates all `4!` stratum orders.  Within an order, each
stratum greedily takes its highest-scoring feasible formation-time pair.  The
order with the largest total score is retained.  Observable feature ranks and
time provide deterministic tie-breaking; storage order is consulted only when
two candidates are exactly identical in every observable feature.

One exact formation-time pair cannot serve two strata.  Different formations
may be selected at the same time so they share a single reference continuation
cache and H=3 no-op arm.  Any two distinct selected times remain at least
`12` pages apart, preserving the V214 trajectory-separation contract.  Thus a
trajectory produces at most four action pairs and between one and four unique
posterior caches.

## Information and claim boundary

The selector may read only the current posterior, current physical/reference
graphs, current link probabilities, two past selected-route pages and exact
current payload proxies.  Target truth, future measurements, future link
draws, future tracking outcomes, absolute sensor identifiers and numeric
formation identifiers do not enter a stratum score.  Truth is used only after
selection to construct paired H=3 value targets.

V218 is a training-data acquisition mechanism.  It is not an online policy,
a tracking result or a conformal guarantee.  The four strata and their feature
directions are frozen before any V218 H=3 outcomes are opened.  The current
best strategy and mechanism tables remain unchanged until a complete online
controller improves the paired metrics.

## Execution order

1. Complete and retain the already-running V214 X36 seed-1301 trajectory as
   need-maximum development evidence.
2. Commit the V218 diagnostics pass-through, protocol and selector.
3. Collect V218 radial M24 seed 1301, then X36 seed 1301, using the same source
   commit and one-pass base route.
4. Generate three-arm H=3 labels only for the selected formation-time pairs.
5. Add the second training seed after the first paired label set confirms that
   all four strata contain distinct positive and negative roles.
6. Freeze the value model on radial M24/X36 before any convoy or relay outcome
   is used for method tuning.

The executable label path is
`runStratifiedDirectGraphThreeArmH3V218.m`.  It first creates the paired
full-payload no-op and one-page withholding continuations.  It then uses the
V216 causal post-fusion synopsis and the V217 position-support and exact-credit
projection to bound complete-label KLA alternatives.  All feasible repair
targets are retained as continuous twelve-coordinate H=3 vectors, including
negative and below-gate rows; the deterministic guard certifies feasibility,
not value.

The repair teacher also requires the current realized request direction and
all source-to-receiver response directions to be available before querying a
forced action.  The subsequent ideal-delivery branch therefore measures the
conditional fusion value of a transport-feasible action; it does not erase an
unobserved packet-loss risk or license deployment without the same transport
projection.

## Causal two-stage learning target

The three alternatives are not a flat classifier.  The withholding decision
is made before communication, while the exact repair candidates and their
post-fusion features exist only after ordinary fusion.  V218 therefore stores
two paired targets:

1. the precommunication head predicts full-payload no-op versus withholding
   from the V215 formation graph features;
2. after a safe withholding action, the post-fusion head predicts every
   complete-label repair relative to the paired withholding-only outcome.

The first head must accept withholding without relying on a later repair.
The second head may improve an already-safe state or abstain, but cannot
retroactively authorize harmful withholding.  This ordering gives a direct
composition argument: if withholding is non-regressive relative to the
reference, repair is non-regressive relative to withholding, and the exact
byte ledger remains nonnegative, the composed action is non-regressive
relative to the reference while respecting its communication budget.
