# V47: causal information-debt scheduling for formation gateways

## Research question

V46 repairs an infeasible registered formation backbone, but its synchronized
B4 candidate still treats every residual message alike.  It sends the whole
residual layer in one step and then remains silent on that layer for three
steps.  This preserves the four-step message count, but it can unnecessarily
remove every cross-formation route at the same time.  The risk grows with the
network because information must traverse more formation hops before it
reaches every sensor.

V47 asks a different question:

> Under the same 37.5% attempted-message saving, which residual messages should
> be sent now so that cross-formation information is refreshed before redundant
> within-formation messages?

The method is not a replacement for LMB-KLA and does not approximate the
fused density.  It is a causal communication scheduler wrapped around the
unchanged LMB filter and fusion implementation.

## Reference decomposition and exact budget

After the V46 current-page repair, the V43 reference gives every receiver one
dominant input and one residual input.  Denote the two directed layers by
\(D_t\) and \(R_t\), with

\[
|D_t|=|R_t|=N.
\]

The full reference therefore attempts \(2N\) messages at every step.  V47
always attempts \(D_t\) and assigns the residual quota

\[
q_\phi =
\left\lfloor \frac{\phi N}{B}\right\rfloor-
\left\lfloor \frac{(\phi-1)N}{B}\right\rfloor,
\qquad B=4,
\]

at phase \(\phi\).  Hence \(\sum_{\phi=1}^{4}q_\phi=N\), including when
\(N\) is not divisible by four.  In every four-step cycle without a safety
fallback, V47 attempts exactly \(5N\) messages instead of \(8N\), a 37.5%
reduction.  This is an exact accounting result, not an output-equivalence
claim.

## Gateway-first residual allocation

V47 partitions \(R_t\) into cross-formation gateway inputs
\(R_t^{\times}\) and within-formation residual inputs \(R_t^{\mathrm{local}}\).
It fills the current quota from \(R_t^{\times}\) first.  Local residual inputs
receive only unused slots.  This changes the sparse page from synchronized
global silence to persistent or interleaved gateway service.

For a residual input \(e=(j\rightarrow i)\), the scheduler records two causal
ages:

- the time since \(e\) was last attempted; and
- the time since \(e\) was last delivered successfully.

It also computes a truth-free disagreement score from the current LMB
posteriors at sensors \(i\) and \(j\), and reads the current link success
probability.  The normalized information-debt score is

\[
s_e(t)=0.45\,a_e^{\mathrm{ack}}(t)
      +0.35\,\delta_e(t)
      +0.20\,r_e(t),
\]

where each term lies in \([0,1]\).  The posterior summary is used only to rank
communication opportunities; the actual KLA still consumes the original LMB
posterior.  No target truth, current delivery random number, future geometry,
future link outcome, or tracking error enters the decision.

## Scale-aware service deadline

Let \(M_t=|R_t^{\times}|\) and let \(q_{\min}=\min_\phi q_\phi\).  V47 defines

\[
H_t=\max\!\left(1,
\left\lceil\frac{M_t}{\max(q_{\min},1)}\right\rceil\right).
\]

Gateway inputs whose attempt age reaches \(H_t\) enter the mandatory set before
the information-debt ranking is applied.  With a fixed gateway set, positive
constant quota, and no fallback, the oldest-service projection visits every
gateway input within a finite scale-derived window.  The implementation uses
immutable physical sensor identifiers only as the final deterministic tie
breaker, so reordering sensor arrays does not change the physical action.

This bound concerns attempted service.  Packet loss prevents a deterministic
guarantee on successful delivery.  Delivery age makes repeated failures more
urgent, while the experiment must separately report attempted and delivered
rolling connectivity.

## Prospective rolling safety and fallback

At every mature step, V47 forms the union of the previous \(H_t-1\) executed
pages and the proposed current page, collapses it to the formation graph, and
checks strong connectivity.  If the prospective attempted-message union is
not strongly connected, V47 executes the full repaired V46 reference page.

The check is causal: the current packet-loss outcome is not yet observable.
It therefore certifies a connected attempted route, not a connected delivered
route.  A fallback also spends more than the nominal sparse quota, so exact
37.5% saving is claimed only for runs with no fallback; fallback rate and
realized communication must always be reported.

## Fusion-weight rule and claim boundary

The reference residual weight is 0.05.  For each residual class, V47 divides
this weight by that class's nominal duty rate and caps the active value at
0.20.  The diagonal is then chosen so that every row sums to one.  This keeps
the nominal class-level residual influence on the same scale as the reference
while avoiding unstable large weights.

The rule does **not** make a sparse sequence equivalent to full LMB-KLA.  KLA
is nonlinear in the posterior density, senders can change after topology
repair, and packet loss changes the realized matrix product.  The defensible
claim is a communication--estimation trade-off supported by structural bounds
and paired tracking experiments.

## Proposed paper contribution

The intended one-sentence contribution is:

> We introduce a causal, scale-aware gateway scheduler that allocates a fixed
> communication budget according to information debt, while an exact topology
> projection preserves executable cross-formation information flow.

The theoretical component should contain only statements proved under explicit
assumptions:

1. exact four-step attempted-message accounting;
2. physical support, row-stochastic weights, and sensor-index equivariance;
3. finite attempted-service time for a stable gateway set; and
4. prospective formation-level rolling connectivity or explicit reference
   fallback.

Tracking improvement, delivered connectivity, and superiority over every
dynamic topology policy remain empirical questions.

## Experiment gates

1. **Runtime contract.** Exercise the real directed filter with physical-UID-
   paired packet-loss draws.  Recompute every attempted and delivered message,
   route hash, weight hash, causal-history boundary, and fallback decision.
2. **Structural screen.** Compare V46 synchronized B4 and V47 at the same
   attempted-message budget on M24 and X36.  Report rolling attempted and
   delivered connectivity, gateway service age, matrix-product contraction,
   fallback rate, and route churn.
3. **Tracking development.** Use paired scenes and seeds to test whether V47
   improves OSPA/GOSPA, cardinality error, consensus disagreement, and recovery
   after formation handoffs.  Do not tune on the final holdout.
4. **Scale test.** Treat D12 as diagnosis, M24 as the main development scale,
   and X36 as the required large-scale holdout.  A method that helps only D12
   does not meet the research goal.
5. **Scene-family test.** Cover radial surround, parallel convoy, linear relay,
   junction crossing, split/merge, and occlusion-relay families.  Freeze the
   method before a leave-one-family-out evaluation.
6. **Ablation.** Remove, one at a time, gateway priority, ACK age, posterior
   disagreement, reliability, the deadline, and the safety fallback.  Compare
   against synchronized B4, uniform round robin, reliability-only scheduling,
   and the full V46 reference.

## Current evidence status

The standalone scheduler and fixed runtime wrapper pass synthetic and real-
geometry contract tests on M24 convoy and X36 radial pages.  Across one
four-step simulated policy window, both scales use exactly \(5N\) messages,
preserve a mature strongly connected formation union, and avoid fallback.
These tests establish implementation feasibility only.  They do not yet show
tracking or consensus improvement.

An adversarial runtime review subsequently closed one evidence-verification
gap: a callback could previously replace the reported service horizon and
re-hash a self-consistent schedule.  The filter now independently replays the
registered projection, V43 reference construction, and V47 scheduler from the
raw callback context before accepting an action.  A coherent horizon-forgery
regression is rejected.

The same review identified a method-level limitation that is not repaired by
the stronger verifier.  V47 ranks edges using all sensors' current posterior
summaries and the global delivery ledger, but its 37.5% number counts only
posterior messages.  Without an explicit centralized controller assumption or
a separately implemented and charged control plane, those inputs are not
available network-wide at zero cost.  Moreover, the first eight-step M24
development-only structural diagnostic gives a worse post-hoc fixed-route
mean-square factor than synchronized B4 (`0.814672` versus `0.784350`) at the
same 240-message budget.  This diagnostic is not a closed-loop tracking score,
but it supplies no reason to pay the missing coordination cost.

V47 is therefore retained as a reproducible negative development result and
must not proceed to a full M24/X36 tracking matrix.  A successor may reuse the
independent evidence replay and gateway-service invariants only if it either
(i) makes its schedule jointly reproducible from genuinely common information,
or (ii) implements, models, and charges every compact-summary, acknowledgment,
and schedule-control transmission.
