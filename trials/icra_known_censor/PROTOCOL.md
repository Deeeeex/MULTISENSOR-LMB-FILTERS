# Known local existence in an already qualified missing-label censor

This is a bounded causal diagnostic of the original nominal GCE failure,
following the complete label genealogy and same-frame reimport census.
Use the same exposed `v2xt_0001`, all 240 frames, two robots, nominal pD=0.9,
and both reliable and intermittent links. Run six fresh original controls
and six complete recursive interventions: GCE, Guarded Scalar, and No-age
under each link condition. Freeze both stages together before any new score.
The controls must reproduce all common fields of the six original nominal
range-preflight runs exactly, except runtime, before intervention execution.

At each local update retain a receiver-local, current-frame scalar cache
before the existing r>0.001 pruning. A fusion output is eligible only if
its label is absent from the retained local input, the same exact label
was predicted and pruned this frame, that local update had a current direct
observation opportunity, and the original fusion already admits the missing
self source as a censor. Replace only that self-source existence operand
0.001 with its known local posterior r<=0.001. No other missing source is
changed. Matching, source participation, remote input and spatial pool are
unchanged. Deleted local Gaussian densities never enter the spatial pool;
the cache is not transmitted. The same intervention applies to all methods.

Use the original clipping [1e-9,1-1e-9] for log-odds. For No-age beta=b.
For GCE and Guarded Scalar recalculate age=sum((q-b)*logit(r)) on the refined
operands and beta=q if age<-1e-12, otherwise b. The output is
sigmoid(sum(beta*logit(r))+I), using the original spatial log integral I.
Eligible events have an absent local Gaussian, so the original current
extra exponents are both zero. Assert this rather than borrowing an extra
likelihood from the discarded local density. This scalar is a known local
marginal including its prior; it is not a new independent measurement.
When the replacement equals the original operand, preserve the exact r.

The regular iteration record retains all original proposal operands and
diagnostics, except columns 7 and 10, which store the actual recursive r.
An additional 25-column event record contains: t, receiver, label(2),
proposal r, actual r, cached prior r, cached posterior r, current flag, pD,
old self operand, remote operand, b(2), q(2), old beta(2), actual beta(2),
old age, actual age, original spatial log integral, old inherited log-odds,
actual inherited log-odds. These semantics are explicit: original beta,
age, r0 and rER in the regular record are proposal diagnostics, not claims
about the final refined calculation.

Before freezing, execute fixtures for all three backends, lower/equal/zero
known r, logit clipping, unavailable cache, no current opportunity,
represented local source, excluded self censor, above-threshold local r,
empty outputs, and exact preservation of every non-r object field and every
non-r proposal-record field. Freeze source and fixture receipts. Every
native job reruns the fixture and all inherited integration fixtures.

Audit every local probability and actual pD, packet and byte accounting,
source matching, full Gaussian mean/covariance/normalizer, and final MAP
extraction and score. Independently join event eligibility from native
pre-prune local records and exact source labels, and reconstruct the actual
scalar equation in Python without calling the native intervention helper.
Keep the inherited numerical tolerances. Check original controls exactly.
Check each intervention's exact causal prefix against its control through
the first altered fusion event, including local inputs and sent messages at
that frame. No tolerance changes after looking at native outcomes.

Report all six paired effects, full OSPA/GOSPA and localization/miss/false
decomposition, cardinality, raw/delivered/padded bytes, all-frame and fixed
GT5/2m/frames53--122 target detection, nearby active components, all-label
and fixed-target same-frame-pruned reimports and consecutive reimports.
Event counts are dependent label events, not physical targets or samples.
Recompute every retained pool and local/fusion pruning balance. Compare
refined GCE with refined No-age as well as each original control.

The sole next-step gate is fixed now: expand this *same* refinement only
if refined GCE strictly improves full OSPA, full GOSPA, and fixed-window
target detections over original GCE in both link conditions, does not lose
full target detections or increase wire bytes in either condition, and
reduces fixed-window target reimports in both conditions. Report each gate,
including any failure; do not change cache eligibility, thresholds, start
frame, duration, or select a subset of methods after outcomes. A pass only
authorizes a separately frozen full-cohort screen; it does not establish
method value or a general benefit. Even if common refinement helps all
methods, attributing a GCE-specific advantage requires comparison against
equally refined No-age. Preserve all failures and keep the paper unchanged.
