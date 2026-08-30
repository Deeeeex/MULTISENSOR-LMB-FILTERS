# V188 budget-recycled residual label routing

## Why V187 is not yet the method

V187 is the first X36 candidate in this branch to improve all four aggregate
objectives relative to the matched static full-posterior reference: mean
E-OSPA `+11.136%`, mean RMSE `+10.718%`, window consensus `+9.834%`, and
attempted communication `+0.160%`.  It still misses the weakest-formation
RMSE gate by `1.050%`, and its protection and repair schedule names the opened
seed-211 times and formations.  It is therefore a mechanism teacher and the
current best balanced development candidate, not an online policy.

Two current-only analytic trigger probes explain why another threshold is not
the answer.  At both t=78 and t=79, formation 5 ranks last under local-to-fused
support debt and fourth under the source-aware common-label opportunity,
although the paired V187 action in formation 5 is useful.  Conversely, simply
running the frozen common-action selector at t=78 in formation 1 produces an
action with safety score `0.751` and minimum scalar risk reduction only
`+0.0002`.  “A safe candidate exists” would therefore trigger too broadly.
The missing signal is the downstream value of a sparse label-state correction
after it propagates through the current information-flow graph.

## Online method

V188 combines two mechanisms that were previously evaluated separately.
The V99-style online admission layer withholds currently harmful or redundant
cross-formation full posteriors while keeping the registered carrier and
control path.  Every avoided payload creates causal communication credit.
The V187 coordinator layer may spend only that accumulated credit on a small
number of complete Bernoulli GM label responses.  The method therefore
reallocates communication from low-value whole posteriors to high-value
edge--label repairs; it does not claim that a light posterior is equivalent to
a full LMB posterior.

The wire protocol is hierarchical.  A 24-byte per-label light synopsis is
used for value prediction and shortlisting.  It carries a four-byte semantic
label key, quantized existence/evidence/support/opportunity scalars, two
float32 position coordinates, and a float32 position-covariance trace.  It
does not carry a full covariance or GM components.  Only shortlisted labels
receive a reconstructable 64-byte moment synopsis.  A selected action then
sends a 48-byte request and the complete mixture-aware Bernoulli label
density.  All headers, synopses, requests, failed deliveries, and responses
are debited from the same credit ledger.  Synopsis exchange has a
deterministic preflight and cannot occur before its cost is affordable.  Of
every newly earned *net* admission saving, twenty percent is moved into a
locked balance that later control or repair traffic cannot spend; cap
overflow is locked as well.  This conservation ledger, rather than a
per-page re-evaluation of the reserve, guarantees nonnegative cumulative
attempted-byte saving.

The learned component only predicts finite-horizon value.  It receives one
row per formation from the frozen 17-feature V188 contract: local support
debt, common-source opportunity, peer agreement, receiver disagreement,
candidate density, normalized physical reach, and one-step causal history.
Absolute time, seed, fixed formation number, numeric label value, truth, and
future information are excluded.  The first baseline is a shared shallow
ranker.  A permutation-equivariant formation GNN is introduced only if M24
and X36 both expose action headroom and the shallow model leaves repeatable
graph-interaction residuals.

## Deterministic projection and theoretical boundary

The value model cannot create an action.  The base admission policy retains
the full-posterior fallback when its own decision is uncertain; after a base
admission decision, the repair projector falls back to that decision without
adding a repair.  A deterministic projector enforces rolling sensor- and
formation-level strong connectivity over the directed formation-diameter
window, admits at most one formation and one complete label per page in the
first version, and rejects any action that exceeds spendable communication
credit.  These rules give two direct properties independent of prediction
quality:

1. cumulative attempted communication never exceeds the full-posterior
   reference and retains the frozen credit reserve; and
2. the effective posterior graph satisfies the registered rolling
   connectivity condition required for repeated KLA information mixing.

They do not bound tracking error, mixture approximation, local Bayes updates,
or model ranking regret.  Those quantities require paired experiments.

## Teacher and scene plan

Each teacher action is evaluated from the same pre-action state with paired
measurements, delivery uniforms, and filter RNG.  Its horizon is one action
page plus the directed formation-cycle diameter: four pages for M24 and six
for X36.  Targets retain E-OSPA, RMSE, worst sensor, worst formation,
consensus, and charged bytes separately.  Windows from one complete
`scene × seed` trajectory cannot cross dataset splits.

The bounded order is:

1. establish projected action headroom on the radial M24/X36 development
   pair;
2. generate grouped teacher trajectories on radial, convoy, and relay scenes;
3. fit and calibrate the shared shallow ranker;
4. add a formation GNN only after a measured interaction residual;
5. freeze the policy before opening merge--split and curved-corridor scenes;
6. retain crossing as a stress test rather than average-case evidence.

## Current authorization

Feature extraction and paired teacher construction are authorized.  Value
model training, GNN training, validation seeds, and paper-facing claims remain
closed until the same projected action family passes the M24 and X36
development gate.  Below-gate experiments remain repository records; the main
progress document continues to show V187 as the current best balanced
candidate.

## Implementation checkpoint

The locked two-stage credit ledger and repair projector pass their focused
Octave invariant test: protected saving is monotone, synopsis traffic is
charged only after preauthorization, and arbitrarily large learned utility
cannot override either safety certificate.  Replaying the original
full-posterior 17-feature block on the opened X36 seed-211 t=79 posterior
required `64.031 s` for one page.  The deployed quantized synopsis cache now
produces the same `6 x 17` contract in `2.684--2.725 s`, a roughly `23.5x`
reduction.  The t=78/t=79 synopsis costs are `40,080 / 39,960 B`, about `1.1%`
of one static full-posterior page in the opened trajectory.  The quantized
common-source signal ranks formation 5 third at t=78 and fourth at t=79,
versus fourth at both pages under the full-moment diagnostic, so the useful
ordering survives the wire approximation.  These results authorize the
bounded M24/X36 projected-action headroom pilot, not model training.

The executable proposal path is now explicit.  A synopsis candidate is
discarded unless every receiver in the formation has nonnegative observable
Bayes-risk reduction from the same physically reachable source.  The routing
key is then resolved back to the exact complete Bernoulli GM object held by
that source.  Its proposal cost includes one 64-byte rich synopsis, one
48-byte request, and one complete response per receiver; the global light
synopsis has already been charged by the page preflight and is not counted a
second time.  The immediate-headroom analyzer freezes these truth-free,
budget-feasible proposals first and reads truth only afterward.  It reports
both whether the action bank contains a jointly positive repair and whether
the current observable proxy selects one.  This isolates action-space
headroom from ranking error before any recursive pilot or learned model is
authorized.
