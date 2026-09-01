# V228 fusion-aware label-edge value controller

## Mechanism decision

V228 was conditional, not an automatic successor.  It was authorized only if
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
decision.  The corrected V226 result at commit `7e21892` contains one such
action at X36 t=50, so the action family is authorized for method development.
This is not yet authorization for an online policy, full-episode result or
paper claim.

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

## Query-first control plane

The corrected t=133 state exposes a protocol bottleneck independently of
value learning.  Withholding F1 is itself safe and earns 10,400 B, but the old
all-active-label synopsis costs 9,480 B.  After the protected-credit reserve,
even a 1,456 B complete label is rejected.  A GNN cannot recover an action
that never enters the feasible bank.

V228 therefore uses a bounded query-first exchange before payload selection.
The beneficiary coordinator chooses at most three labels from its ordinary
current posterior and broadcasts only their four-byte keys.  Every remote
participant is conservatively charged one 16-byte query header plus the three
keys, and one 16-byte response header plus one 20-byte record per key.  The
response record contains quantized existence and evidence, two position
coordinates, position-covariance trace and the complete-label payload size.
For 20 participants the worst-case charge is 1,976 B.  The corrected t=133
F1 banks have 15 participants, so their exact frozen bound is 1,456 B rather
than the old 9,480/9,840 B all-label synopses.  With the 1,456 B smallest
complete-label payload, both F1->F5 and F1->F6 teacher rows retain 7,488 B of
net attempted-byte saving before delivery outcomes.

The query is paid before a source is selected; the complete Bernoulli-GM label
payload is paid separately before exact eta projection.  Twenty percent of
new communication credit remains protected.  Numeric label identifiers are
routing keys, not learned features.  The deterministic byte planner is
`planFusionAwareQueryFirstExchangeV228`; the paired H=3 headroom runner is
`runQueryFirstBoundedTransferHeadroomV228`.

The online controller may fetch only one complete label per page.  It ranks
source-label candidates from the compact query records and advertised payload
size, fetches the highest calibrated candidate, and then applies the exact
receiver-wise eta projector.  Full-mixture or realized eta features cannot be
used for pre-fetch ranking because they do not exist until the payload has
already been transmitted.  If every receiver rejects the payload, the page
falls back to ordinary fusion and still pays the bytes; it does not try a
second complete label.  This closes the earlier inconsistency between a
three-candidate exact shortlist and the one-payload communication contract.

This headroom run still uses teacher-selected donor/source/label rows.  Its
purpose is narrower and necessary: determine whether the now-affordable,
exactly projected action space contains enough multi-objective value to merit
training an online rule, ridge model or GNN.

Before opening the query-first H=3 outcomes, V228 freezes a separate material
headroom gate.  A candidate must first be compositionally safe.  It must then
show at least `0.50%` incremental tracking gain in E-OSPA or RMSE, at least
`0.50%` joint tracking gain, at least `0.25%` joint window or terminal
consensus gain, and at least `0.25%` attempted-byte saving.  The corrected
V226 t=50 action remains useful evidence but fails this material gate because
its best consensus gain is only `0.052%`.  Therefore it authorizes V228 design
but not grouped trajectory collection or model fitting by itself.

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

The communication side has a separate deterministic certificate.  For `P`
participants, at most `K` queried labels, query header/key sizes `h_q,b_q` and
response header/record sizes `h_r,b_r`, the worst-case control charge is

`B_query = (P-1) [(h_q + K b_q) + (h_r + K b_r)]`.

If suppressing the donor page earns `C` attempted bytes, the protected-credit
fraction is `rho`, and the selected complete label costs `B_payload`, the
payload is admitted only when

`B_query + B_payload <= (1-rho) C`.

Therefore the realized attempted-byte saving before delivery effects is
`C-B_query-B_payload >= rho C > 0`.  The value model cannot weaken this
inequality.  For the corrected t=133 F1 bank, `P=15`, `K=3`, `rho=0.2`,
`C=10,400 B` and `B_query=B_payload=1,456 B`, giving a certified `7,488 B`
net saving and `5,408 B` of spendable feasibility margin.

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

At runtime the value model ranks a bounded causal bank built from compact
query records.  It selects at most one source-label payload using the highest
calibrated vector margin, pays and fetches that complete payload, and then
performs exact receiver-wise projection.  If projection rejects the payload,
the controller returns the frozen carrier no-op without a second payload
attempt on that page.

## Evidence sequence and stop rules

1. **V226 mechanism gate (passed).**  Require a non-regressive donor-only state,
   positive incremental transfer gains, positive joint gains on aggregate
   E-OSPA, RMSE and both consistency measures, and attempted-byte saving.
   Otherwise stop this line.  Keep merely joint-positive actions as mechanism
   diagnostics, not as authorization evidence.
2. **V228 query-first headroom.**  Re-evaluate the safe t=133 donor with the
   bounded query charge and complete payload.  Require at least one action to
   pass the frozen material headroom gate before constructing a learning
   dataset; merely positive actions remain mechanism records.
3. **V227 carrier choice.**  Compare static and dynamic carrier routes using
   identical scene, seed, measurements, link uniforms, message count and full
   mixture-aware KLA.  Freeze the carrier before constructing V228 data.
4. **Training only.**  Use radial M24/X36 seeds 1301 and 1303, grouped by the
   complete scene-seed trajectory.  Do not split states from one trajectory
   across train and validation folds.
5. **Calibration.**  Use the registered 1409 and 1423 groups only after the
   architecture and feature set are frozen.  Calibrate vector residuals and
   no-op abstention, not a truth-dependent runtime gate.
6. **Primary evaluation.**  M24 and X36 seeds 1511 and 1523 must each show
   lower E-OSPA and RMSE, better consistency and fewer attempted bytes.  Report
   the weakest sensor, formation and recovery tail as well as the mean.
7. **No-retune transfer.**  Run convoy and relay first, then crossing,
   braided-handover, merge-split, target-overlap and curved-corridor, followed
   by X48.  A radial-only result remains development evidence.

The paper method is the fusion-aware action and deterministic LMB-KLA
projection.  Request-response control and the GNN estimator are implementation
choices already represented in adjacent literature and are not claimed as the
novelty by themselves.
