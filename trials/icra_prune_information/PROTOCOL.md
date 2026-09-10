# Current pruning information: paired recursive causal experiment

This is a new information-availability intervention, motivated by the verified
one-frame return after a receiver-only refinement. The closed receiver-only
known-censor rule remains closed. No cache lifetime, censor threshold, admission
condition, fusion time or exposed window is tuned here.

The exposed `v2xt_0001` case is run from frame 1 through 240, with nominal pD 0.9,
the original reliable and intermittent directed delivery masks, and all three
backends: GCE, Guarded Scalar, No-age. Six new receiver-only controls must exactly
reproduce the six completed known-censor trajectories, apart from runtime and
new empty recorder fields. Six communicated-information trajectories are frozen
at the same time and may start only after those controls pass their full audit.
The six original trajectories are also reported as already exposed references.

At every send, append a binary trailer to the existing message. Its four
little-endian float64 header fields are `[73190501, sender, frame, count]`.
Each row contains four float64 fields `[birth_frame, birth_location, local_r,
actual_pD]`. Include every current local prediction whose updated r is at most
0.001 and which has a current direct opportunity (actual pD > 0). Include no
deleted spatial density, previous local r, truth, or prior-frame report. Empty
trailers cost 32 bytes. Rows cost 32 bytes each. Encode, concatenate, decode and
validate actual uint8 bytes. Only a delivered current packet exposes its trailer
to the receiver. Source, time, count, length, bounds, integer labels and duplicate
keys are checked. All local caches and received reports are cleared each frame.

Apply the old fusion first. For an existing output label with exactly one absent
Gaussian input, replace that input's 0.001 censor operand only if its original
existence weight is positive and the same exact label has a qualified current
pruned scalar. The receiver uses its own current cache or the decoded sender
trailer, according to the absent input. Retained-source matching is unchanged;
pruned scalar reports do not create or align labels. The original spatial pool,
Gaussian mean/covariance, source weights b and q, and absence eligibility remain.
The original absence rule already disables extra current exponents in this case.
Recompute the inherited history branch using the new logits: No-age uses b;
GCE/Guarded Scalar use q iff `(q-b) dot logits < -1e-12`, otherwise b. The final
existence is sigmoid(beta dot logits + original spatial log normalizer), using
the unchanged [1e-9, 1-1e-9] logit clipping. Equality to 0.001 is an exact no-op.
Record original proposals and actual modified r separately. This substitutes a
known posterior operand; it does not add a new independent negative likelihood.

All attempted/delivered payload bytes use the concatenated actual packet size.
The existing cost is ceil(packet bytes / 16384) * 16384 per attempted directed
send plus 256 control bytes per frame, including failed sends. Report overhead
and padding separately; no free negative side channel is allowed. The larger
three-backend information sets must be identical in construction, although their
recursive trajectories naturally produce different rows.

Before native outcomes, verify the binary parser and all three scalar backends,
both absent-input positions, clipping, eligibility, non-r preservation, and
local-only equivalence. Independently reconstruct every trailer from native
sender update records, every used operand from current delivered bytes, the
actual scalar equation, all positive evidence and Gaussians, matching, prediction
and pruning balances, MAP extraction, scores, and control parity. Report complete
robot-frame counts and distinguish observed label events from physical targets.

The previously exposed diagnostic window stays frames 53–122 and target ID 5;
target detection uses one-to-one 2 m matching against all truth. Census every
remote-only retained label over the whole trajectory, partitioned into same-frame
local prune, first arrival, and return after an absent-prediction gap. Gate uses
same-frame prune plus gap return in the target window, so moving returns one
frame later cannot satisfy it by itself. This descriptive diagnostic never
enters the online fusion or message selector.

Expansion gate, fixed before any new outcome: for GCE under BOTH link conditions,
(1) full OSPA and GOSPA each improve by at least 1% versus the receiver-only
control; (2) full OSPA and GOSPA are each strictly lower than communicated No-age;
(3) fixed-window target detections strictly increase and full target detections
do not decrease versus the receiver-only control; (4) fixed-window target label
recurrences strictly decrease; (5) attempted padded wire bytes are no more than
105% of the receiver-only control. The 5% cap is a predeclared small-overhead
budget, not an outcome-selected tolerance. Also report comparisons against the
original GCE and No-age. Guarded Scalar is a shared-information comparator and
cannot replace failed GCE after seeing results.

If any gate fails, close this exact current-pruned-scalar communication family;
do not rescue it with thresholds, longer report persistence, scene/window
selection or a different favored backend. If all pass, only a separately frozen
whole-cohort screen is authorized by the evidence; no general method-value or
paper claim follows from this already exposed single case. No paper or shared
production fusion implementation is changed by this isolated trial.
