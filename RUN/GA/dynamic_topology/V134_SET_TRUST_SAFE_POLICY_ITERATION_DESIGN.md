# V134: scale-aware binary posterior-admission scheduling

## Boundary from the preceding Adaptive-KLA work

The preceding Adaptive-KLA paper studies how received posteriors should be
weighted: it adapts KLA fusion weights using posterior and communication
quality. V134 therefore cannot claim a continuous trust factor as a new
method. Its decision variable is instead whether a complete cross-formation
posterior is admitted on each future page. Nominal KLA weights remain frozen.
An admitted input uses its original route weight, and an omitted input is not
sent beyond a charged control synopsis. Intermediate weights and learned
fusion-weight allocation are forbidden by the protocol. The runtime also
forbids stale-posterior substitution: zero admission means that no current or
cached posterior from that controlled edge participates in KLA.

This boundary is empirical as well as semantic. Persistent abstention is a
diagnostic protection upper bound only and cannot pass the method gate. A
gate-eligible action must restore every omitted input and avoid regression in
a contraction-certified tail after full restoration. Later evaluation must include a 2-by-2
factorial comparison of static versus binary admission and fixed versus
Adaptive-KLA weights, so any gain can be attributed to message scheduling
rather than to repeating the earlier weight-allocation mechanism.

## Why the one-page pulse is rejected

V133 remains useful as an atomic counterfactual: it measures the effect of withholding one receiver formation's incoming cross-formation posterior at one state. It is not a closed-loop policy. Its examples begin from static-carrier states, while repeated deployment changes the posterior state distribution and creates interactions between formations.

The earlier V134 draft proposed a one-page set pulse followed immediately by full-posterior recovery. Existing tracking evidence contradicts that action. Useful protection takes several pages to propagate through the formation cycle; abrupt restoration produces a return shock, while persistent zero-trust protection eventually creates recursive downstream harm. The decision object must therefore include both the protected formation set and its finite-horizon reintegration schedule.

## Action: a set and a binary admission sequence

At an eligible page, the controller chooses a nested receiver-formation set and a binary admission sequence

\[
    \mathcal A_k = \{(A_{k+h}, a_{k+h})\}_{h=0}^{H-1},
    \qquad a_{k+h} \in \{0,1\} .
\]

Only the selected cross-residual inputs into formations in \(A_{k+h}\) are affected. Physical reachability, the frozen static carrier, attempted message opportunities and all unselected fusion inputs remain unchanged.

- \(a=0\): the sender transmits only a charged control synopsis; the receiver does not consume that posterior.
- \(a=1\): the sender transmits its complete current mixture-aware LMB posterior and the receiver uses its frozen nominal KLA input weight.

No Gaussian-mixture component is collapsed when \(a=1\). This is not the V129 light-posterior experiment: V129 changed both representation and weight, whereas V134 preserves the full mixture and changes only temporal admission. Every non-reference action therefore has a direct wire-level interpretation.

The frozen missing-input semantics determine where an omitted input's weight goes. Under `renormalize`, the remaining inputs are renormalized. Under `self`, removed neighbor mass is transferred to the receiver's self input. Both produce a row-stochastic KLA vector. These are missing-message semantics, not learned fusion-weight allocation.

## Why the old recovery window was invalid

The earlier draft used the formation-cycle diameter as both the protection
time and the post-reentry maturity time. That confuses propagation between
formations with propagation over all sensor states. A structural calculation
at the registered pilot anchors showed that the old complete windows were not
mean-square contractive: the worst-case squared factors were about 2.48 for
M24 over 10 pages and 2.59 for X36 over 16 pages. Therefore an apparent
short-window gain could not support a claim that the network had recovered.

Using pilot seed 1601 and the registered time-varying routes and link
probabilities, a fully
restored tail of `2(N-1)` pages gives squared contraction factors
0.5861/0.6536 for the M24 CW/CCW carriers and 0.8003/0.8915 for the X36
carriers. All four are below the preregistered 0.90 target. These values are
structural development evidence, not tracking-error guarantees.

## Scale-aware schedules

Let \(D_F\) be the directed diameter of the frozen formation cycle and let \(F\) be the number of formations. The first bank uses two sequence families for every nested set:

1. persistent zero admission over the full horizon, retained only to measure the protection upper bound;
2. zero admission for \(D_F\) pages, followed by reverse-risk-order reentry of one formation per page; all remaining pages use full admission so the mature tail is measured after complete restoration.

Let \(N\) be the sensor count and set the post-reentry recovery tail to
\(R_N=2(N-1)\). The complete horizon is

\[
    H = D_F + F + R_N.
\]

M24 uses \(D_F=3,F=4,R_N=46,H=53\), while X36 uses
\(D_F=5,F=6,R_N=70,H=81\). The M24 pilot remains anchored at \(t=95\)
and ends at \(t=147\). The X36 anchor moves from the nominal focus midpoint
to the latest complete start, \(t=80\), so its recovery tail ends at the
registered trajectory boundary \(t=160\). Reverse risk order restores the
least risky member first and keeps the most strongly protected formation
omitted longest.

The recovery certificate computes exact expected centered-L2 propagation
for the registered route sequence under independent delivery draws. Under
common positive support this also controls KLA set-density log-ratio
disagreement. It does not bound local Bayes updates, Bernoulli overlap
normalizers, label-support loss, mixture approximation or pruning, and hence
is not a standalone tracking-safety theorem.

## Candidate-set construction

Current observable posterior risk, disagreement, observation quality and downstream reach rank the formations. A pure nested-prefix bank would expose only the highest-ranked formation as a singleton: on X36, five of six formations could never be protected alone. Failure of such a bank would therefore confound an ineffective admission mechanism with inadequate set coverage.

The v4 bank instead contains every rank-space singleton, every rank-space pair, and one nested risk prefix for each size from three through \(F\). This covers direct effects and the smallest non-additive interaction without enumerating all \(2^F\) subsets. The deployable set count is

\[
    F + \binom{F}{2} + (F-2) = \frac{F(F+1)}{2} + F - 2,
\]

which gives 12 sets on M24 and 25 on X36. Physical formation identifiers are not part of the construction: singleton, pair and prefix names refer only to observable rank positions, so relabeling formations preserves the bank. Each set receives the same staggered binary-reentry sequence. Persistent-zero variants remain diagnostic-only and are not run in the first pilot.

Sets of size three or larger remain nested prefixes because higher-order exhaustive enumeration would add combinatorial cost before pairwise interaction has shown value. If the pair-complete bank passes both scales, later teacher data may test whether repeatable higher-order residuals justify a richer set encoder; validation outcomes cannot be used to expand the bank.

## Pre-learning stop gate

Learning is not authorized until the action space itself beats the stronger full-trajectory CW/CCW static carrier selected independently for each scale. On M24 and X36 separately, the best deployable sequence must achieve:

- at least 5% paired E-OSPA gain while any posterior is actually omitted;
- no full-window or certified-recovery-tail E-OSPA regression;
- no sensor or formation regression in the intervention, full, or recovery window;
- positive attempted-byte saving after control metadata;
- a fully restored `2(N-1)`-page tail whose registered expected KLA mixing factor is at most 0.90.

If either scale fails, V134 remains an experiment record and the learner is not built. This prevents a model from hiding an action space with insufficient causal headroom.

After both scale pilots are finalized, `finalizeSetTrustSequenceV134JointGate`
recomputes every action gate from the stored component metrics and recovery
certificate. M24 and X36 must pass independently; their gains cannot be
averaged to compensate for a failed scale. A passing joint artifact authorizes
only the next interpretable on-policy teacher-development stage. It does not
authorize a GNN, a validation claim or a main-document result.

## Teacher and on-policy iteration

Once the headroom gate passes, every candidate sequence is evaluated from the same state with paired measurements, delivery uniforms, filter RNG, route and nominal weights. The teacher retains intervention, full-window and recovery-tail E-OSPA, worst sensor and formation, reachable downstream effects, disagreement and bytes as separate outcomes.

The first model is a shared scale-normalized additive set model with calibrated residual bounds. Complete trajectories are then rolled out with the actual policy; fresh counterfactual labels are generated at the states it visits and aggregated with earlier data. The empty static action is the fallback outside calibrated support. A GNN or learned set encoder is considered only if repeatable set-interaction residuals remain on both scales and the richer model improves held-out safe utility.

## Evaluation and reporting order

1. Freeze the stronger full-trajectory static direction for M24 and X36.
2. Measure pair-complete, rank-equivariant sequence headroom.
3. Stop if the joint cross-scale gate fails.
4. Run conservative on-policy dataset aggregation.
5. Evaluate once on unseen radial seeds.
6. Freeze the radial policy, then evaluate convoy and relay as held-out geometries; crossing remains an extreme stress test.

Below-gate candidates, parameter screens and isolated bright spots stay in repository experiment records. The main progress document receives only a stable method decision or complete aggregate evidence that passes the registered joint gate; it does not list every failed iteration.

The exact post-learning evidence hierarchy is frozen separately in
`V134_MULTISTYLE_GENERALIZATION_PROTOCOL.md`. One scale-normalized checkpoint
must first pass unseen radial seeds at M24 and X36, then transfer without
retuning to the geometry-qualified convoy and relay scenes. Each scale and
each transfer style must pass independently. Crossing is a non-gating failure
envelope, while the other available scene families remain development assets
until they receive a separate held-out geometry contract.
