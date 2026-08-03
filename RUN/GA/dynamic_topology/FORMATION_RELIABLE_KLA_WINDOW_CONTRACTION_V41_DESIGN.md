# V41 reliability-aware KLA window propagation certificates

## Research decision

V39 showed that exact current-step posterior disagreement is strongly biased
against the requested sparse actions: all five requested-safe single-bundle
actions in the first three M24 source caches increased that one-round score.
V40 isolated one weight confound by preserving the reference conditional
weights when a residual message is absent.  The intervention was exact in all
16 checked routes, but failed its frozen advancement gate: only 2/5
requested-safe actions lowered the score and only 1/4 cases contained a safe,
positive-protection, nonpositive-risk action.  Neither result supplies the
missing multi-step principle.

V41 separates two questions that previous versions mixed together:

1. how a random, reliable route window propagates disagreement already
   present at the sensors; and
2. how local Bayes updates, spatial conflict, label loss and approximate LMB
   fusion inject new disagreement.

Graph theory can certify the first term.  It cannot, by itself, predict
tracking error or the second term.  V41 therefore provides structural
propagation certificates that can later constrain a posterior-dependent or
data-driven value model.  It is not a graph-only tracking policy.

## Auxiliary single-event Dobrushin certificate

Let `M_t` be the realized row-stochastic fusion matrix at step `t`, using
receiver rows and sender columns.  Choose a causal set of required delivered
edges and let `L_t` contain each raw self weight plus the raw weights of those
required edges.  For both registered missing-message rules, `renormalize` and
`self`, delivery of all required edges implies `M_t >= L_t` entrywise.  Thus

```text
M_h ... M_1 >= B_h := L_h ... L_1.
```

With

```text
beta(B_h) = min_(i != j) sum_k min(B_h(i,k), B_h(j,k)),
```

the required-delivery event gives

```text
delta(M_h ... M_1) <= 1 - beta(B_h).
```

If the required receiver-sender-time deliveries are independent and their
event probability is `q`, then

```text
E[delta(M_h ... M_1)] <= 1 - q beta(B_h).
```

The theorem and implementation are correct, but the automatic schedule is
only a greedy lower-bound constructor.  A registered four-node counterexample
contains one high-quality ring input per row (`w=0.30`, `p=0.99`) and weak
complete-graph shortcuts (`w=0.001`, `p=0.01`).  Greedy direct broadcast gives
`q beta = 9.9e-8`; a caller-supplied three-step ring schedule gives
`0.026198073`, over 260,000 times larger.  The test suite freezes this
counterexample.

The first geometry-only M24/X36 diagnostic also makes the scale problem
explicit.  At the focus-start current page for seed 41, the reference event
gain is approximately `5.06e-6` for M24 and `9.21e-9` for X36.  The resulting
upper bounds are nearly one and are unsuitable for candidate ranking.  The
single-event result remains a rigorous auxiliary diagnostic, not the primary
runtime signal.

The coefficient of the product of expected pages is reported only as a
diagnostic.  Convexity gives the wrong inequality direction for treating it
as an upper bound on the expected realized Dobrushin coefficient.

## Primary exact mean-square window certificate

Define the centering projector

```text
Pi = I - 11'/N
```

and the realized product `P = M_H ... M_1`.  Under independent
receiver-sender-time delivery indicators, V41 computes

```text
Q = E[P' Pi P]
```

exactly without enumerating all joint network delivery outcomes.

For one page, let `mu_i = E[m_i]` be effective receiver row `i` and let
`C_i = E[m_i' m_i] - mu_i' mu_i`.  Different receiver rows are independent,
so for any deterministic symmetric matrix `S`,

```text
E[M' S M] = E[M]' S E[M] + sum_i S(i,i) C_i.
```

Starting from `S_H = Pi`, the backward recursion

```text
S_(t-1) = E[M_t' S_t M_t]
```

returns `S_0 = Q`.  Its cost is polynomial in network size and exponential
only in each receiver's bounded incoming count, which is at most four in the
registered protocol.

Let

```text
rho_H = lambda_max(Pi Q Pi).
```

Then, for every deterministic node-value vector `x`,

```text
E[||Pi P x||_2^2] <= rho_H ||Pi x||_2^2.
```

Because

```text
||Pi x||_2^2 = (1/N) sum_(i<j) (x_i-x_j)^2,
```

`rho_H` is a worst-initial-direction expected pairwise mean-square
propagation factor.  `rho_H < 1` certifies strict expected contraction;
`rho_H > 1` means the route can transiently amplify centered L2 disagreement
in a worst direction.  It must never be reported as a passed safety gate.

The backward recursion also returns every suffix factor.  For a fixed route
whose future delivery draws are independent of current state and future
disturbances, Minkowski's inequality gives an RMS propagation bound for
pre- or post-mixing additive disturbances.  This supplies the structural
coefficients needed by a later recursion such as

```text
z_t = M_t z_(t-1) + xi_t,
```

but V41 does not yet bound `xi_t` itself.

Exact small-network enumeration verifies `Q` for both `renormalize` and
`self`.  The property suite covers multiple incoming neighbors, fixed and
probabilistic delivery, random time-varying pages, deterministic delivery,
identity/disconnected pages and malformed 4-D inputs.  Additional numerical
fixtures verify cancellation of the finite-hypothesis KLA normalizer, exhibit
a nonzero Bernoulli spatial-overlap `log eta`, and show that correlated packet
delivery can make the independence-based factor nonconservative.

## Exact KLA scope

For exact KLA with common positive support, fix two complete finite-set
hypotheses `X` and `Y` and define at node `i`

```text
z_i = log f_i(X) - log f_i(Y).
```

The KLA normalization cancels in this ratio, so a fusion-only iteration obeys
the same linear mixing recursion.  The mean-square certificate therefore
applies pointwise to complete set-density log ratios under common support.

It does **not** imply that marginal Bernoulli existence log odds mix linearly.
For one Bernoulli component,

```text
logit(r_bar) = sum_i omega_i logit(r_i) + log eta,
```

where `eta` is the spatial-overlap normalizer.  Marginal existence log odds
therefore contain a posterior-dependent term.  New measurements, differing
label support, Gaussian-mixture approximation and pruning add further
disturbances.  These are precisely the effects that a proper-metric bound or
calibrated value model must control next.

## Runtime semantic boundary

The certificate is valid only when the effective row distribution matches the
implemented model.  The runtime validator therefore requires:

- `alwaysHeavy` attempts with the link gate disabled;
- no stale-neighbor substitution;
- no mixed payload mode;
- no mode-aware fusion reweighting;
- unit non-static weight factor and no self/non-static weight caps; and
- one of the modeled `renormalize` or `self` missing-message rules.

A separate communication validator rejects forced delivery, deterministic
outage schedules and any realized `linkUniforms` in the policy context.  It
also requires an explicit attestation binding the supported receiver path to
the repository's independent uniform draw per sender-receiver-time attempt.
Missing fields do not pass by default, because the existing context sanitizer
removes runtime-only fields.  This is a registered simulator contract, not an
empirical independence test on a realized random array.

The route probe copies only the current physical graph, registered base graph,
past selected-topology pages, current drop-probability page and primitive
runtime semantics.  It replaces posterior contents by empty placeholders
before constructing routes.  Common-label support is still a separate KLA
claim condition, not something the runtime validator infers.

## Scale-aware horizon calibration

The original `N-1` window is only a graph broadcast-depth heuristic.  It does
not control transient amplification for directed, non-normal mixing matrices.
The first seed-41 geometry-only diagnostic found:

| Scale | `rho_(N-1)` | First `rho<1` | First `rho<=0.9` | `rho_(2(N-1))` | `rho_(4(N-1))` |
|:--|--:|--:|--:|--:|--:|
| M24 | 1.196453 | 28 | 31 | 0.493151 | 0.070411 |
| X36 | 2.010545 | 89 | 95 | 1.324488 | 0.356447 |

Thus M24 and X36 do not share a fixed useful horizon, and even `2(N-1)` is
insufficient on X36.  The frozen development rule now repeats only the
current reference route and current reliability page, searches up to
`4(N-1)`, and selects the first horizon with reference `rho<=0.90`.  The 0.90
target is an explicit ten-percent expected-energy contraction target.  It was
frozen from geometry-only development before opening any new tracking
continuation or target truth.  If the target is absent within the cap, the
probe reports `targetHorizon=NaN`, retains the maximum horizon only for a
reference diagnostic, and suppresses all candidate evaluation.

Calibration uses the runtime-configured missing-neighbor rule after the
receiver semantics pass the explicit validator.  The current source
configuration resolves to `renormalize`; the implementation records that
value instead of treating it as an unstated universal default.

Operationally, one horizon page denotes one ordinary online fusion update
after a tracking step; V41 does not insert extra fusion rounds inside a step.
The candidate action changes the first page only, followed by `H-1` registered
reference pages.  Its window communication count is therefore reported
explicitly, including the delta from an all-reference window.  For X36,
`H=95` means a 95-step propagation horizon, not 95 free consensus iterations.
Repeating the current reliability page that far is a causal development
approximation, and the unbounded local-update disturbance prevents it from
being interpreted as an executable safety guarantee.

At the calibrated seed-41 horizons, all four M24 single-bundle suspensions
worsen `rho` by `0.0126--0.0157`.  On X36, formation 6 improves it from
`0.889080` to `0.884617`, formation 2 is nearly neutral, and the other four
worsen it.  This is a nonconstant structural signal and a counterexample to
the idea that every sparse action must weaken finite-window propagation.  It
is still not tracking evidence and does not identify which action is useful
after posterior disturbances are included.

## Method implication

The promising method architecture is now:

1. construct a finite, physically feasible action bank that changes
   cross-formation mixing mass;
2. calibrate a current, scale-aware reference mixing horizon from the exact
   mean-square recursion;
3. retain the structural factor and its suffix disturbance coefficients as
   analytic features or constraints;
4. estimate posterior/innovation disturbance and multi-step task value with a
   proper metric or a data-driven model; and
5. project any learned choice back onto physical, communication, label-
   retention and registered structural constraints.

The current evidence does not yet justify a hard requirement
`rho_candidate <= rho_reference`: all M24 suspensions would be rejected, and
the structural factor omits the posterior-protection benefit that motivated
the action.  The next theoretical step is to quantify the disturbance term,
not to tune a tolerance against tracking outcomes.

## Code and evidence contract

- `computeDobrushinErgodicityCoefficient` implements the standard
  coefficient identity.
- `computeReliableKlaWindowContractionCertificate` implements the auxiliary
  single-event bound.
- `computeExpectedEffectiveMixingPageMoments` enumerates exact per-receiver
  first and second row moments.
- `computeReliableKlaWindowMeanSquareContractionCertificate` implements the
  exact backward quadratic recursion and suffix factors.
- `computeStaticReliableKlaMeanSquareHorizonProfile` extracts every repeated-
  page horizon from one maximum-window recursion.
- `validateReliableKlaMixingRuntimeSemantics` rejects unmodeled receiver-side
  weight transformations.
- `validateReliableKlaCommunicationRuntimeSemantics` rejects forced delivery,
  outages and realized delivery draws that invalidate or leak the independent
  expectation model.
- `buildFormationReliableKlaWindowContractionRouteProbe` evaluates a fixed
  candidate-once/reference-recovery plan without reading posterior contents.
- `buildFormationReliableKlaAdaptiveWindowRouteProbe` calibrates the
  reference horizon and evaluates the action bank at that common horizon.
- the two certificate tests compare against full delivery enumeration; the
  route-probe test covers M24/X36 dimensions, runtime isolation and adaptive
  calibration.  Nonbinary graphs/masks, negative weights, four-dimensional
  histories and missing communication attestations fail closed.

The current geometry runner deliberately materializes the complete planned
sensor geometry and planned link-probability schedule before slicing the
current page, and therefore labels itself
`formalRuntimeObservableBoundaryPassed=false`.  After the frozen 15-case M24
source batch closes, a separate manifest must bind current pages and the
registered mission plan without target, measurement, truth or realized link-
uniform access.  Formal tracking remains sealed until that manifest and a
posterior-disturbance protocol both pass.

## Current evidence boundary

V41 establishes an implementation-checked structural theorem and a
geometry-only development signal.  It does not establish M24 or X36 tracking
improvement, does not authorize a tracking continuation, and does not support
a paper performance claim.  V40 remains a valid negative result at commit
`13bd6d1`; its failure is not erased by this redesign.
