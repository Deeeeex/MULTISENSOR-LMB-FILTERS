# V55 full-reference label rate-distortion theory

## One-sentence contribution

V55 formulates cross-formation LMB posterior exchange as a
context-conditioned, label-wise rate-distortion problem: under the same byte
budget as full V46 messages, it chooses which GM label objects to transmit so
that the installed receiver fusion remains close to the result of receiving
all current messages, while the V46 backbone preserves the required network
information path.

This sentence is a candidate paper claim, not yet an empirical conclusion.
It becomes defensible only if the selection-only arm preserves or improves
tracking on M24 and X36 while reducing bytes.

## Why the reference must be the full V46 input set

For receiver `j` and label `ell`, partition the current inputs into the local
receiver label, fixed incoming V46 labels `F_j,ell`, and selectable
cross-residual labels `S_j,ell`.  The reference is

`q_ref(j,ell) = KLA(q_j,ell, F_j,ell, S_j,ell)`.

For one selected subset `A_j,ell` of the cross-residual inputs, the candidate
is

`q_A(j,ell) = KLA(q_j,ell, F_j,ell, A_j,ell)`.

Every candidate contains the same local and fixed inputs.  The teacher varies
only the cross-residual label objects that consume the optional payload
budget.  This conditioning matters because KLA normalization can change the
relative value of two optional inputs even when the additional fixed inputs
are common to both candidates.

The reference contains every present label object that a full V46 message
would supply.  Current evidence type is not permission to delete a predicted
track: lack of a detection opportunity and reliable evidence of absence are
different statements.  Evidence remains useful as an online feature and for
the optional receiver-protection ablation, but it does not define the V55
selection-only reference.

## Exact LMB decomposition

Write one Bernoulli label density as `q=(r,p)`, where `r` is the existence
probability and `p` is the conditional spatial density.  For matched label
spaces, the reference-to-candidate LMB divergence decomposes over labels:

`D_KL(pi_ref || pi_A) = sum_ell d_ell`,

where

`d_ell = (1-r_ref) log((1-r_ref)/(1-r_A))`

`      + r_ref log(r_ref/r_A)`

`      + r_ref D_KL(p_ref || p_A)`.

This decomposition is exact for the LMB representation.  It explains why the
communication decision may be organized label by label without claiming that
sender effects are additive: multiple sender inputs for the same label still
interact through KLA and must be evaluated jointly.

The installed posteriors use Gaussian mixtures.  V55 evaluates the spatial GM
term with deterministic positive-weight cubature.  Therefore the Bernoulli
decomposition and combinatorial optimization are exact for the option table,
but the numerical value assigned to each spatial option is an approximation
to GM KLD.  The paper must not call the overall density distortion exact.

## Shared-edge byte model

Let `z_i,ell` indicate that sender `i` transmits label `ell`, and let `y_i`
indicate that at least one label activates its payload edge.  The charged
bytes are

`B(z) = B_synopsis + sum_i h_i y_i + sum_i,ell b_i,ell z_i,ell`,

subject to `y_i >= z_i,ell` for every label.  Here `h_i` is the message header
cost and `b_i,ell` is the serialized GM label cost.  The current offline
experiment pays a compact synopsis for every selectable edge and constrains
synopsis plus selected payload bytes not to exceed the paired full-message
bytes:

`B(z) <= B_full,V46`.

The shared header makes independent per-label knapsack decisions incorrect.
Activating one label can make additional labels on the same edge cheaper than
labels on an unused edge.

## Exact selector for an enumerated option table

For each label, V55 enumerates all sender subsets permitted by the bounded
receiver degree.  Each option records its approximate distortion, variable
label bytes, and activated-edge mask.  The selector propagates states of the
form

`(processed label index, active-edge mask, accumulated bytes, distortion)`.

After each label, it retains the Pareto frontier separately for every
active-edge mask: a state is discarded only when another state with the same
mask uses no more bytes and has no larger distortion.  The final feasible
state with minimum distortion is therefore globally optimal for the
enumerated option table and the installed byte model.  No byte rounding or
greedy sender ranking is used.

The exactness claim is deliberately narrow:

- exact: joint choice across labels, shared edge headers, and the supplied
  option distortions;
- approximate: GM spatial KLD values inside those options;
- not optimized: E-OSPA, cardinality error, or target truth;
- nondeployable: the offline teacher reads complete sender posteriors before
  deciding which payloads would have been sent.

## Role of the V46 backbone

V55 does not learn connectivity from scratch.  The V46 dominant backbone and
nonselective inputs remain fixed, and V55 compresses only current
cross-formation residual payloads.  This separation gives the method a clear
division of responsibility:

- structural routing preserves the effective information path and bounds the
  number of optional sender inputs;
- label rate-distortion allocation spends the remaining bytes on posterior
  content that best reproduces full-message receiver fusion.

The tracking contribution must come from this combination.  Consensus alone
is not a success criterion because V54 showed that the network can agree more
closely on a worse posterior.

## Projection is an ablation, not part of the core claim

The optional receiver-protection layer constrains the existence log-odds of a
locally supported label.  Given a fixed lower bound, clamping only the
existence probability to the boundary is the KL-nearest Bernoulli correction
to the unconstrained fused label.  This result does not justify the bound
value, and it does not prove that applying the correction improves tracking.

V54 invoked this correction 6,356 times on X36 and worsened cardinality error.
V55 therefore treats projection as a separate causal factor:

- projection-only: full V46 payloads plus direct existence clamp;
- selection-only: full-reference rate-distortion selection without a clamp;
- combined: the same selection plus the direct clamp.

The paper should make label rate-distortion selection the core method if the
selection-only arm succeeds.  Projection should be retained only if the
combined arm adds stable tracking benefit without cardinality bias.

## Deployable form and remaining theoretical boundary

The current teacher establishes offline headroom.  A deployable receiver-side
policy requires two communication subrounds within one fusion period:

1. fixed backbone payloads and compact cross-residual synopses arrive;
2. the receiver evaluates or predicts label-subset values in the observed
   fixed context;
3. the receiver requests selected GM labels;
4. senders return the requested labels and the receiver fuses them.

Final accounting must add request bytes and state the latency assumption.  A
set GNN may approximate the option distortion or ranking using only received
synopses, fixed-context summaries, link history, and byte costs.  Target
truth, future measurements, future link outcomes, and unsent GM components
are forbidden online features.

## Evidence required before manuscript promotion

The method is not ready for a positive paper claim until all of the following
hold:

1. The selection-only offline teacher saves attempted bytes and improves or
   preserves full-horizon E-OSPA, focus-window E-OSPA, cardinality error, and
   worst-sensor behavior on both M24 and X36.
2. The result repeats across radial, convoy, merge-split, and curved-corridor
   scenes with disjoint development and held-out seeds.
3. A deployable synopsis-conditioned policy recovers a substantial fraction
   of offline-teacher benefit after request/control bytes are charged.
4. Projection attribution supports either removing the existence floor or
   retaining it with a documented, stable benefit.

Until these gates pass, the defensible contribution is a theoretically
structured candidate method and a negative result about evidence-gated
receiver projection, not a validated tracking improvement.

## X36 outcome and claim status

The first full-reference selection-only X36 run failed the empirical gate.
It saved 3.79% of total attempted bytes but changed full-horizon E-OSPA,
focus-window E-OSPA, and cardinality error by -1.37%, -1.47%, and -2.89%.
The direct saving on the selected residual path was only 2.05%, and candidate
runtime increased by 35.1%.  The positive paper claim proposed at the start
of this document is therefore not supported by the current method.

This negative result exposes a scope mismatch in the candidate story.  A
teacher that minimizes divergence from full V46 fusion is suitable for
posterior-preserving compression, but it has no tracking-aligned mechanism
for improving on V46.  The combined selection-plus-projection experiment is
not justified.  Only the projection-only causal arm remains before deciding
whether to recast the work as robust fusion or abandon the payload branch in
favor of tracking-aligned topology control.
