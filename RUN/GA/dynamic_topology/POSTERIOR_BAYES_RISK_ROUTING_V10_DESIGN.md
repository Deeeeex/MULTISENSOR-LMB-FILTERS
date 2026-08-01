# Posterior Bayes-risk routing v10 design

## Motivation

The v9 joint source-trust controller scores one dynamic source map and then
linearly interpolates that score across the 0.30, 0.50, and 0.70 trust
levels. This makes the action value only indirectly related to the
post-fusion estimator.

The frozen formation-FoV M24 validation makes the failure mode concrete.
V9 improved mean E-OSPA by 8.85% and saved 2.59% of attempted bytes, but
only four of five seeds improved and aggregate worst-node E-OSPA regressed
by 3.86%. The exact global mean disagreement score still favored the
selected action in every seed. Formation-level attribution then showed
that gains in one formation could compensate for substantial losses in
another. The next objective must therefore score actual post-fusion modes
per receiver and enforce formation/tail protection rather than relying on
one globally averaged risk.

V10 replaces that value model with a truth-free posterior decision risk.
The exact one-step disagreement calculation and deterministic graph,
payload, and connectivity projection remain separate safety layers.

## Label-wise posterior risk

For a Bernoulli label with existence probability `r`, position covariance
`P`, OSPA order two, and cutoff `c`, compare two point decisions:

- report no object: `R_empty = r`;
- report one object at the posterior mean:
  `R_point = (1-r) + r min(trace(P)/c^2, 1)`.

The label risk is `min(R_empty, R_point)`. The empty-decision term is exact.
For the point decision, concavity of the capped squared loss gives

`E[min(||X-E[X]||^2/c^2,1)] <= min(trace(P)/c^2,1)`.

Consequently, `R_point` is a conservative upper bound for that restricted
point decision, rather than an arbitrary score. The resulting label proxy
is dimensionless, bounded, permutation invariant, and contains the
between-component spread already present in the moment-matched covariance.
Link uncertainty is integrated over the same receiver outcome distribution
already produced by the exact one-round teacher.

For every formation and trust mode, receiver risks are aggregated with a
mean-tail objective. Mode value is the reduction relative to the
registered reference mode. Consequently, each trust level receives its
own post-fusion value rather than a linear interpolation of the full-trust
edge score.

## Safety and claim boundary

This quantity is an OSPA-inspired, label-wise posterior risk upper-bound
proxy, not exact multi-object E-OSPA. The upper-bound statement applies to
the restricted per-label decision above; it does not turn the mean-tail
multi-label aggregation into an exact multi-object risk. A confidently
biased posterior may have low internal risk, so the exact current-posterior
disagreement guard remains mandatory. No truth, future measurement, future
E-OSPA, or numeric label identity is used online.

The first implementation milestone is deliberately limited to pure risk,
outcome integration, and formation aggregation functions with synthetic
tests. It is not connected to the frozen v9 validation. The completed M24
pair identifies receiver/formation masking as a real failure mode; X36 is
still required before choosing the exact integration and fallback policy.

## Planned evidence gates

1. On opened development states, posterior Bayes-risk advantage must rank
   the exact short-horizon tracking teacher better than the v9 analytic
   score without reducing safe-action recall.
2. Receiver-level and formation-tail value must prevent a positive
   formation sum from masking a harmful receiver.
3. After freezing the design, fresh paired M24 and X36 runs must each reach
   at least 5% mean E-OSPA gain, 5/5 positive seeds, no aggregate worst-node
   or consensus regression, and at least 2% communication saving.
4. A later deployable model must distill both edge value and the expensive
   exact risk teacher; replacing only the value score with a GNN is
   insufficient for online execution.
