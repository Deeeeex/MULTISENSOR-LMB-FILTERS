# V134: finite-horizon binary posterior-admission scheduling

## Boundary from the preceding Adaptive-KLA work

The preceding Adaptive-KLA paper studies how received posteriors should be
weighted: it adapts KLA fusion weights using posterior and communication
quality. V134 therefore cannot claim a continuous trust factor as a new
method. Its decision variable is instead whether a complete cross-formation
posterior is admitted on each future page. Nominal KLA weights remain frozen.
An admitted input uses its original route weight, and an omitted input is not
sent beyond a charged control synopsis. Intermediate weights and learned
fusion-weight allocation are forbidden by the protocol.

This boundary is empirical as well as semantic. Persistent abstention is a
diagnostic protection upper bound only and cannot pass the method gate. A
gate-eligible action must restore every omitted input and retain its gain in a
mature tail after full restoration. Later evaluation must include a 2-by-2
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

## Scale-aware schedules

Let \(D_F\) be the directed diameter of the frozen formation cycle and let \(F\) be the number of formations. The first bank uses two sequence families for every nested set:

1. persistent zero admission over the full horizon, retained only to measure the protection upper bound;
2. zero admission for \(D_F\) pages, followed by reverse-risk-order reentry of one formation per page; all remaining pages use full admission so the mature tail is measured after complete restoration.

Thus \(H=2D_F+F\): M24 uses \(D_F=3,F=4,H=10\), and X36 uses \(D_F=5,F=6,H=16\). This gives the intervention one complete formation-propagation time before reintegration and one complete propagation time after the largest set is fully restored. Reverse risk order restores the least risky member first and keeps the most strongly protected formation omitted longest.

## Candidate-set construction

Current observable posterior risk, disagreement, observation quality and downstream reach rank the formations. The bank contains the empty reference plus nested prefixes of that ranking. With \(F\) formations it has \(1+2F\) actions rather than \(2^F\) subsets. The persistent-zero member of each prefix is diagnostic-only; only the staggered binary-reentry member is deployable and gate-eligible.

The full subset bank is retained only as an offline action-space oracle on a small number of registered development states. It answers whether the nested construction leaves important set interactions unexplained; it is not available to the deployed controller.

## Pre-learning stop gate

Learning is not authorized until the action space itself beats the stronger full-trajectory CW/CCW static carrier selected independently for each scale. On M24 and X36 separately, the best deployable sequence must achieve:

- at least 5% paired mean E-OSPA gain;
- at least 5% paired mature-window E-OSPA gain;
- no registered sensor or formation regression;
- positive attempted-byte saving after control metadata.

If either scale fails, V134 remains an experiment record and the learner is not built. This prevents a model from hiding an action space with insufficient causal headroom.

## Teacher and on-policy iteration

Once the headroom gate passes, every candidate sequence is evaluated from the same state with paired measurements, delivery uniforms, filter RNG, route and nominal weights. The teacher retains mean and mature E-OSPA, worst sensor and formation, reachable downstream effects, disagreement and bytes as separate outcomes.

The first model is a shared scale-normalized additive set model with calibrated residual bounds. Complete trajectories are then rolled out with the actual policy; fresh counterfactual labels are generated at the states it visits and aggregated with earlier data. The empty static action is the fallback outside calibrated support. A GNN or learned set encoder is considered only if repeatable set-interaction residuals remain on both scales and the richer model improves held-out safe utility.

## Evaluation and reporting order

1. Freeze the stronger full-trajectory static direction for M24 and X36.
2. Measure full-subset oracle and nested-sequence deployable headroom.
3. Stop if the joint cross-scale gate fails.
4. Run conservative on-policy dataset aggregation.
5. Evaluate once on unseen radial seeds.
6. Freeze the radial policy, then evaluate convoy and relay as held-out geometries; crossing remains an extreme stress test.

Below-gate candidates, parameter screens and isolated bright spots stay in repository experiment records. The main progress document receives only a stable method decision or complete aggregate evidence that passes the registered joint gate; it does not list every failed iteration.
