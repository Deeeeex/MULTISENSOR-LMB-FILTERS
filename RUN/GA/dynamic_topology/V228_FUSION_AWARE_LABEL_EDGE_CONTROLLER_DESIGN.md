# V228 fusion-aware label-edge value controller

## Decision before opening the corrected result

V228 is conditional, not an automatic successor.  It is authorized only if
the corrected V226 pilot contains a compositionally safe action: withholding
alone must be non-regressive on aggregate E-OSPA, position RMSE, window
consensus and terminal consensus; the label transfer must improve all four
coordinates relative to that donor-only state; the joint action must improve
all four relative to the ordinary reference; and attempted bytes must fall.
This matters because a lost or receiver-rejected label transfer otherwise
exposes the donor-only state.  A jointly positive action that repairs a harmful
donor remains a mechanism upper bound but cannot authorize the safe online
controller.

The registered local-tail tolerances remain the stricter paper-promotion gate;
they do not hide a useful compositionally safe mechanism from the method
decision.  If V226 has no such action, the label-edge action family is dropped
and no GNN is trained.

The physical carrier is frozen separately from the V227 paired full-episode
comparison.  The dynamic carrier is retained only if it is non-regressive at
both M24 and X36 without increasing attempted bytes.  Otherwise V228 uses the
registered static carrier and creates dynamics only in the effective
label-wise KLA input graph.  This prevents a generic route change from being
credited for a label-aware gain.

## Controlled object

One V228 composite action contains:

1. at most one formation whose ordinary incoming full-posterior page is
   withheld to create exact byte credit; and
2. at most one beneficiary-formation, source-sensor and label tuple whose
   complete labeled Bernoulli Gaussian-mixture posterior is offered to the
   receivers in that formation.

Each receiver independently transfers the largest registered safe fraction of
one positive non-self KLA input weight to the offered source.  The receiver's
self input is never replaced, total label-wise KLA weight is unchanged, and a
receiver that rejects every fraction executes the ordinary fusion.  The full
label payload is charged before projection; a small accepted weight therefore
does not manufacture a communication saving.

This is not generic feature communication.  The learned decision changes the
inputs of one ordinary label-wise LMB-KLA operation while retaining the full
mixture density that later filter recursion consumes.

## The deterministic guarantee

For one label, the Bernoulli KLA existence identity used by the active fusion
path is

`logit(r_bar) = sum_i omega_i logit(r_i) + log(eta)`.

Let `L0` be the log-odds from ordinary fusion and `L1` the candidate log-odds.
The frozen V226 projector accepts a receiver only when

`L0 - 0.25 <= L1 <= L0 + 0.25`,

and, if the ordinary label is MAP-positive, additionally requires `L1 >= 0`.
Consequently:

- the fused odds can change by at most a factor of `exp(0.25) = 1.2841`;
- the global probability change is at most
  `tanh(0.25 / 4) = 0.06242`;
- an ordinary MAP-positive label cannot cross below the 0.5 extraction
  threshold; and
- the ordinary result is recovered exactly whenever no fraction passes.

The proof uses only monotonicity of the logistic map.  The exact maximum of
`|sigmoid(z + delta) - sigmoid(z)|` for `|delta| <= epsilon` is
`tanh(epsilon / 4)`.  Weight conservation follows directly because a fraction
`alpha` moves `alpha * omega_j` from one non-self input to the offered source,
leaving the weight sum and self weight unchanged.

These are algorithmic guarantees relative to the ordinary result produced by
the installed mixture-aware KLA approximation.  They are not an E-OSPA, RMSE
or spatial-density guarantee.  That limitation is why the H=3 paired teacher
and closed-loop evaluation remain necessary.

## Learned component

The model only estimates value.  It cannot create a physical link, alter the
byte ledger, relax the KLA envelope or suppress the no-op fallback.

The shared graph contains one node per formation, carrier and physical edges,
and a token for the proposed source-label action.  Runtime inputs are causal:
current posterior summaries, current graph and delivery history, compact
source-label synopsis, payload size and available credit.  Sensor and
formation identifiers are metadata, not features.  Counts are normalized so
the same model can run on M24, X36 and X48.

The target is the twelve-coordinate H=3 gain vector already registered by
V213, measured against the same-state no-op.  Truth and future outcomes may
construct offline targets but are unavailable online.  The first comparison
is an observable rule, followed by action-only ridge and graph-feature ridge.
A two-round formation GNN is retained only if grouped unseen-trajectory error
and closed-loop paired outcomes improve over both ridge baselines.

At runtime the value model ranks a bounded causal bank.  Exact projection is
performed only for the top three candidates, and the highest calibrated
vector-margin action that passes all deterministic checks is executed.  If no
candidate passes, the controller returns the frozen carrier no-op.

## Evidence sequence and stop rules

1. **V226 mechanism gate.**  Require a non-regressive donor-only state,
   positive incremental transfer gains, positive joint gains on aggregate
   E-OSPA, RMSE and both consistency measures, and attempted-byte saving.
   Otherwise stop this line.  Keep merely joint-positive actions as mechanism
   diagnostics, not as authorization evidence.
2. **V227 carrier choice.**  Compare static and dynamic carrier routes using
   identical scene, seed, measurements, link uniforms, message count and full
   mixture-aware KLA.  Freeze the carrier before constructing V228 data.
3. **Training only.**  Use radial M24/X36 seeds 1301 and 1303, grouped by the
   complete scene-seed trajectory.  Do not split states from one trajectory
   across train and validation folds.
4. **Calibration.**  Use the registered 1409 and 1423 groups only after the
   architecture and feature set are frozen.  Calibrate vector residuals and
   no-op abstention, not a truth-dependent runtime gate.
5. **Primary evaluation.**  M24 and X36 seeds 1511 and 1523 must each show
   lower E-OSPA and RMSE, better consistency and fewer attempted bytes.  Report
   the weakest sensor, formation and recovery tail as well as the mean.
6. **No-retune transfer.**  Run convoy and relay first, then crossing,
   braided-handover, merge-split, target-overlap and curved-corridor, followed
   by X48.  A radial-only result remains development evidence.

The paper method is the fusion-aware action and deterministic LMB-KLA
projection.  Request-response control and the GNN estimator are implementation
choices already represented in adjacent literature and are not claimed as the
novelty by themselves.
