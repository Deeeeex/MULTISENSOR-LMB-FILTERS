# V226 corrected bounded label-edge transfer finding

## Decision

The corrected mixture-aware LMB-KLA code line contains one receiver-specific
label-edge transfer that is compositionally safe and byte-positive.  This
authorizes construction of the V228 value controller, but the effect is too
small to support the paper's final significance claim.

V228 method development may proceed only under the stagewise gate frozen in
`codex/v228-fusion-aware-controller` commit `c3d4cac`: donor-only must be
non-regressive, the label transfer must improve all four core coordinates
relative to donor-only, the joint action must improve them relative to
ordinary fusion, and attempted bytes must fall.  A merely joint-positive
repair of a harmful donor is not authorization evidence.

## Corrected paired result

- Scene / seed / window: `x36-formation-fov / 1301 / t=50`, H=3.
- Corrected trajectory commit: `19a2484`.
- V226 pilot commit: `5fe8dd8`.
- Action: withhold F2's incoming full-posterior page; transfer S23 label
  `[19,15]` to F5 through receiver-specific bounded non-self KLA weight
  transfer.
- Donor-only gains: exactly zero on aggregate E-OSPA, RMSE, window consensus
  and terminal consensus; attempted bytes fall by 0.800%.
- Incremental and joint gains: E-OSPA `+0.005533%`, RMSE `+1.94245%`, window
  consensus `+0.035342%`, terminal consensus `+0.051860%`, attempted-byte
  saving `+0.929092%`.
- Candidate / ordinary-reference values: E-OSPA `122.166701 / 122.173461`,
  RMSE `10.797458 / 11.011348`, attempted bytes
  `2,329,268 / 2,351,112`.
- Worst sensor gains: E-OSPA `+0.000%`, RMSE `+6.239%`.
- Minimum formation gains: E-OSPA approximately `0.000%`, RMSE `-0.0028%`.
- All twelve registered V213 risk tolerances pass.  The minimum registered
  margin is `+0.005533%`; the minimum local-tail margin is `+1.4972%`.

The result passes the risk gate but not the separate paper-promotion gate:
E-OSPA `0.006% < 5%`, RMSE `1.942% < 5%`, consensus `0.035% < 2%`, and
communication `0.929% < 1%`.  It is a teacher-selected training-window
mechanism, not an online policy, full-episode comparison or generalization
result.

## Structural finding at the higher-need window

At `t=133`, withholding F1 alone improves E-OSPA by `0.184%`, RMSE by
`0.465%`, window consensus by `0.547%`, terminal consensus by `0.647%`, and
attempted bytes by `0.404%`, with no reported worst-sensor or minimum-formation
regression.  It is therefore a genuinely safe posterior-admission action.

However, its `10,400 B` admission credit cannot fund the current all-label
synopsis plus even the smallest complete-label response.  The F5 proposal uses
`9,480 B` of synopsis and a `1,456 B` payload, exceeding the available credit
by `536 B`; every t=133 F1 candidate is rejected before tracking evaluation.
The second selected donor, F3, is not stagewise safe and also has no feasible
candidate after control overhead.

This separates two problems that a stronger GNN cannot solve by itself:

1. **Value estimation:** decide when suppressing one incoming posterior and
   moving one label edge is beneficial.
2. **Action protocol:** expose affordable candidates without spending most of
   the earned credit on an all-label synopsis.

The V228 successor should retain the exact KLA and byte projections, but use
query-first discovery: a beneficiary sends a bounded set of label keys, and
only matching sources return compact risk records.  The earlier V222 bound is
`1,976 B` for three keys and twenty participants, versus roughly `9.5--12.3
kB` for the current all-label synopsis.  This is an implementation mechanism,
not the novelty claim; the paper contribution remains the fusion-aware
selection and safe projection of complete labeled Bernoulli GM posteriors.

## Evidence boundary

The authoritative report is
`corrected_mechanism_pilot/training/x36_formation_fov_seed1301/CORRECTED_BOUNDED_TRANSFER_MECHANISM_PILOT_V226.md`.
Raw screen MAT files remain local because the two donor captures alone occupy
about 172 MB.  The committed package retains the compact result, candidate
banks, reports and logs needed to audit the numerical conclusion.  V227 still
must supply the matched fixed-static-route baseline, after which V228 may
construct grouped M24/X36 training data without opening evaluation seeds.
