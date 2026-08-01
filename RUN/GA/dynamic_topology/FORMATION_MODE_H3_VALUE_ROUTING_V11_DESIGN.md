# Formation-local H=3 value routing v11

## Status and evidence boundary

This is an opened-development redesign. It is not a registered closed-loop
arm and it cannot support an M24 or X36 validation claim. The five M24
predecision states used to motivate it were already opened under the frozen
formation-FoV v9 study.

The v10 posterior-risk prototype is not promoted. At the first topology
divergence of seeds 83, 89, 97, 101, and 103, its truth-free one-step score
did not consistently separate formations that later improved from formations
that later regressed. A threshold-only adjustment would therefore optimize a
misaligned target rather than repair the target.

The first v11 end-to-end smoke case gives a concrete falsifier. At M24 seed
83, time 71, the formation-1 dynamic action with trust 0.30 has a negative
v10 posterior objective and is rejected by that proxy. A paired H=3 return,
using the same current posterior, measurements, link uniforms, and filter RNG,
instead gives:

- mean E-OSPA gain: +9.428%;
- worst-sensor E-OSPA gain: +0.000%;
- consensus-OSPA gain: approximately +4.02%;
- attempted-byte saving: approximately +0.86%.

This is development evidence that an H=3 value target contains information
that the one-step handcrafted proxy misses. It does not establish that the
same action remains beneficial under a learned closed-loop policy.

## Method hypothesis

The method separates value estimation from hard feasibility.

Offline, a privileged teacher scores short counterfactual returns. At each
training state, the action bank contains the registered reference plus actions
that change exactly one formation to one candidate trust mode for the first
step. The following two steps return to the registered fixed reference. Truth
and future measurements are used only to compute the H=3 targets.

Online, a size-shared formation value model receives only current observable
features. It predicts a conservative lower bound for tracking value and a
tail-risk estimate for every formation-mode pair. A separate exact projector
combines the local actions while enforcing physical reachability, payload,
one-step disagreement, rolling connectivity, and reference fallback.

The existing posterior-risk quantities are retained as observable features
and diagnostics. They are no longer treated as sufficient value labels.

## Formation-local intervention bank

For F formations and M modes, the initial teacher bank contains

1 + F(M - 1)

actions rather than M^F joint actions. M24 has four formations and four modes,
so the bank has 13 actions; X36 has six formations and therefore 19 actions.
Each nonreference action replaces only the receiver rows of one formation in
the reference adjacency and fusion-weight matrices. This keeps the action
semantics explicit and permits shared training examples across formation and
network sizes.

The initial additive representation ignores interactions between simultaneous
formation changes. The exact joint projector still evaluates joint structural
constraints. If additive value headroom passes but joint selections fail, a
sparse pair-intervention extension will estimate only the largest residual
interactions rather than returning to exhaustive M^F rollout labels.

## Teacher targets

Every paired H=3 arm stores the raw quantities needed to define or revise the
learning target:

- network mean E-OSPA gain relative to the reference;
- worst-sensor E-OSPA gain;
- per-formation mean E-OSPA gains and their minimum;
- consensus-OSPA gain;
- attempted and delivered payload bytes;
- selected and delivered rolling-B3 connectivity checks.

The first learning target should not collapse these fields prematurely into
one scalar. The preflight must first measure oracle headroom under at least two
explicit acceptance rules:

1. mean-value rule: positive network mean gain under the reference payload;
2. tail-safe rule: positive mean gain with no formation below the registered
   development tolerance.

Only after headroom is established should communication savings enter the
selector objective. This prevents a byte term from making a weak tracking
teacher appear useful.

## Observable runtime features

No seed identifier, absolute label identifier, truth, future measurement, or
future outcome may enter the deployable feature block. Candidate features are
shared across formations and may include:

- normalized formation size and network size;
- current LMB existence, covariance, and mixture-complexity summaries;
- reference-relative posterior-risk objective;
- receiver mean-tail disagreement change;
- link success and sender-payload summaries on changed rows;
- graph overlap, source load, and two-step rolling-history summaries;
- candidate trust weight and reference-relative payload change.

Permutation-invariant aggregation within each formation and shared parameters
across formations are required for M24-to-X36 transfer. A message-passing model
is justified only after the same feature/target contract shows nontrivial
headroom with a simpler regularized baseline.

## Conservative selection and theory target

Let q(g,m) denote the unknown H=3 value of mode m for formation g relative to
the reference and let L(g,m) be a calibrated lower confidence bound. The
runtime action set admits a dynamic mode only when L(g,m) is positive. The
joint projector then maximizes the sum of admitted values under exact hard
constraints and may always choose the all-reference action.

The intended theory contribution is conditional rather than an unconditional
tracking guarantee:

- hard topology, payload, and rolling-connectivity constraints hold exactly
  by construction;
- if the simultaneous lower bounds cover their true formation values, every
  admitted local mode has nonnegative H=3 value relative to reference;
- with uniformly bounded value-estimation error epsilon, an additive feasible
  action selected from F formations has a standard finite-action regret bound
  proportional to F epsilon, plus an explicitly measured interaction residual.

Coverage and interaction residuals must be measured on development data and
rechecked on untouched validation splits. They must not be presented as exact
tracking safety without those conditions.

## Experimental gates

### Opened teacher-signal preflight

1. Complete all payload-compliant local modes at the five first-divergence M24
   states.
2. Audit common random numbers, exact first-action replay, and selected B3
   connectivity for every arm.
3. Require useful oracle headroom on multiple seeds and at least one
   non-outlier state; report false positives and false negatives of v10.
4. Reject or expand the action bank if the best tail-safe action is reference
   on nearly every state.

### Training and model selection

1. Generate disjoint D12, M24, and X36 training/development scenarios.
2. Fit a regularized scalar baseline before a message-passing model.
3. Calibrate lower bounds by scenario-size group and report empirical coverage.
4. Freeze features, candidate construction, model parameters, confidence
   margin, and the exact projector before opening validation pairs.

### Fresh paired validation

The final claim requires new M24 and X36 scenario-seed pairs. The candidate
must show a clear aggregate tracking gain, stable seed-level behavior,
nonnegative registered tail/worst-node gates, consensus safety, and real byte
savings against the same fixed reference. Opened M24 states, v9 X36
development pairs, and the currently running frozen v9 formation-FoV pairs do
not satisfy this gate for v11.
