# V74 receiver direct task-risk design

V73 aligns source scoring with the formal mixture-aware heavy-fusion
receiver, but its net opportunity still measures only supported existence
gain minus supported existence loss. V72 shows why that is incomplete: X36
formation 5 is already worse in the intervention step, whereas formation 4
improves immediately and degrades later. Immediate task risk and temporal
propagation must therefore be tested separately.

V74 remains source-only. At each opened M24/X36 anchor it reconstructs the
mixture-aware V73 reference and all feasible single-formation and joint
formation routes. For every route it enumerates the current independent link
delivery outcomes and performs the same heavy-payload fusion and missing-input
weight normalization as the runtime receiver.

Two existing truth-free quantities are reused:

1. posterior Bayes risk, which upper-bounds the better of an empty decision
   and a point decision at the posterior mean using Bernoulli existence and
   position covariance;
2. exact one-round posterior-summary disagreement, which compares existence
   and covariance-normalized spatial moments across every receiver pair.

A candidate passes the direct gate only when every affected formation has
nonnegative robust mean-tail Bayes-risk improvement and both mean and upper
quartile network disagreement do not increase. These are sign constraints,
not thresholds tuned to V72 tracking outcomes.

V74 diagnoses whether this direct layer rejects the immediately harmful X36
formation-5 action while preserving a useful candidate. It does not execute a
route or authorize tracking. If the direct layer is discriminative, temporal
propagation is added afterward for actions such as X36 formation 4 that look
good immediately but deteriorate later.
