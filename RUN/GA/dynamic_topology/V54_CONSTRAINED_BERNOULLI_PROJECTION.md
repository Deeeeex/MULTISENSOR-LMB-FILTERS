# V54 constrained Bernoulli projection

## Scope

V54 protects a receiver label only when that receiver has current positive
measurement-association evidence for the label.  Let the receiver posterior
existence be `r_R`, and let the unconstrained distributed fusion result for
the same label be the Bernoulli density

`q_bar = Bernoulli(r_bar, p_bar)`.

The protection rule permits a maximum receiver log-odds drop `delta`:

`logit(r) >= logit(r_R) - delta`.

The current development setting is `delta = log(4)`, so the protected fused
existence odds cannot be less than one quarter of the receiver's locally
supported odds.  This numerical value is a frozen design choice for the X36
oracle gate, not a universal constant or a consequence of KLA theory.

## Proposition: the existence-only clamp is KL-nearest

Define the feasible lower bound

`rho = sigmoid(logit(r_R) - delta)`.

Among all Bernoulli RFS densities `q = Bernoulli(r, p)` satisfying
`r >= rho`, the solution of

`min D_KL(q || q_bar)`

is

`p_star = p_bar`,

`r_star = max(r_bar, rho)`.

Therefore, when the unconstrained fused label violates the receiver-protection
bound, retaining its spatial density and clamping only its existence
probability to `rho` is the exact KL-nearest constrained correction.

### Derivation

For one Bernoulli RFS label,

`D_KL(q || q_bar)`

`= (1-r) log((1-r)/(1-r_bar))`

`  + r log(r/r_bar) + r D_KL(p || p_bar)`.

For every fixed `r > 0`, the spatial term is minimized uniquely at
`p = p_bar`.  The remaining binary Bernoulli KL has derivative

`dD/dr = logit(r) - logit(r_bar)`

and second derivative

`d^2D/dr^2 = 1/(r(1-r)) > 0`.

It is strictly convex on `(0,1)` and has its unconstrained minimum at
`r = r_bar`.  Projection onto the interval `[rho, 1]` consequently leaves
`r_bar` unchanged when it is feasible and otherwise moves it to the boundary
`rho`.  The implementation clips probabilities away from zero and one before
forming log-odds, which is the numerical counterpart of this interior-domain
derivation.

## What the proposition does and does not justify

The proposition justifies the final existence-only clamp in
`projectReceiverSafeFusionRetention.m`.  It also explains why reverting the
spatial density to the receiver-only posterior would be an unnecessarily
large correction: `p_bar` already minimizes the stated constrained KL
problem.

It does **not** prove that the preceding greedy removal of selective sender
inputs is globally optimal.  That step is a causal heuristic that first tries
to eliminate harmful received label inputs while leaving the frozen V46
backbone unchanged.  Its transmitted bytes remain charged.  A globally
optimal joint sender-subset choice belongs to the offline oracle, and later to
the set-GNN approximation only if the oracle clears the tracking gate.

The proposition also does not imply better tracking.  The constraint can
preserve direct positive evidence, but frequent active clamps can introduce
positive cardinality bias.  The paired X36 experiment must decide this from
full-horizon E-OSPA, cardinality error, focus-window behavior, total bytes,
and unresolved constraint violations.

## Claim boundary for the paper

If V54 passes the empirical gates, the defensible theoretical claim is
narrow:

> Given a receiver-supported log-odds floor, the V54 post-fusion correction is
> the KL-nearest feasible Bernoulli label density and preserves the
> unconstrained fused spatial posterior.

The paper should not call the entire V54 routing algorithm KL-optimal unless
the joint sender-subset problem is separately solved or bounded.
