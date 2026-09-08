# Structural method iteration: pooled prior and current local evidence

Started 2026-09-08 under the continuing objective to obtain a convincing
main experiment before rewriting the paper. The completed IR/CR/CGR study
remains immutable. Its nine development and six former reserved sequences
are now all seen data for method development. The old three-candidate limit
belongs to that completed round; this is a separately recorded new round.

## Decision gate

A new paper-facing method needs a complete, frozen comparison against its
own structural ablations, original ER, no-age, MIL-AM and TC on matched
inputs. A favorable development mean alone is insufficient. Preserve every
complete sequence, adverse result, false/missed-target decomposition,
common localization support and communication cost. A final candidate must
be fixed before using new reserved tracking outcomes. Keep data previously
used for diagnosis out of that final selection check. Only then revise the
paper around the result actually supported; do not select frames or edit
scores to obtain the desired main result.

## Hypothesis and bounded first comparison

IR moved normalized weights from the entire posterior to the current
update, but it still averages the local new evidence. CR trades reduced
false cost for missed targets, while a two-hit bit does not exclude
persistent false measurements. Test whether averaging current independent
sensor updates unnecessarily weakens both positive and negative evidence.

For a matched label, write l_j=logit(r_j^+) and
d_j=logit(r_j^+)-logit(r_j^-), recomputed at each local update. Keep the same
spatial KLA pool and log normalizer h. Let b be ordinary eligible weights
and q be original ER eligible weights, with the unchanged .25 floor and
5-second kernel. For at least two eligible represented sources, with no
eligible missing-label censor, test these two fixed forms:

    JE:   z = sum_j b_j (l_j-d_j) + sum_j d_j + h
    JE-R: z = sum_j q_j (l_j-d_j) + sum_j d_j + h

In every other label case, JE falls back to the existing no-age rule and
JE-R to original ER. Thus a single represented source cannot use this new
correction to override a qualified missing-label censor. No threshold is
fitted. Do not run a nominal age decay on current increments: every nonzero
local update is already current, so such an ablation can be vacuous.

This pools inherited predicted existence and accumulates current effective
updates once per distinct received source. It is a controlled approximate
Bernoulli rule: the local LMB association update is not an independently
calibrated likelihood ratio, the priors may differ, and the spatial pool
is held fixed. No exact centralized Bayes or unknown-correlation guarantee
is claimed. Conditional independence motivates accumulation only; correlated
detector errors and recursive overconfidence are specific failure risks.

Only one extra float64 per Bernoulli is needed, using the existing IR packet.
Every direct update overwrites the increment; a received increment cannot
be relayed as a new local one. The tracker receives no truth. Keep detection,
births, poses, motion, pruning, matching and all radio draws unchanged.

Analytic checks: equal common prior and spatial density recovers the scalar
product-likelihood update; two negative updates reinforce absence; opposing
updates cancel; one input and eligible censor recover the named fallback;
untouched priors abstain; zero increments reduce to the corresponding
posterior pool; spatial means/covariances and packet round trips remain
unchanged. Reproduce original ER on the same instrumented runner before
attributing any metric change to the candidate.

Use sequences 0000--0002 as development preflight, then complete the same
nine-sequence development cohort if the implementation checks pass. Retain
both JE and JE-R regardless of which is better. Further structural changes
require a separate amendment before their outcomes are inspected.

## Prior work boundary

Separating prior and likelihood consensus is established prior work, not a
new contribution claimed here. Fantacci's 2015 dissertation explicitly
motivates separate pooling to avoid underweighting new information:
https://flore.unifi.it/handle/2158/1003256 . This experiment tests a narrow
Bernoulli-existence specialization with existing eligibility/censor logic;
it does not reproduce the dissertation's complete algorithm.
The single-source Bernoulli update background is also available in
https://arxiv.org/abs/2606.09573v1 . Neither source validates this candidate
or the fixed-probability real-detection replay used here.

## Implementation preflight correction

The single-source unit check failed before any candidate tracking run:
the core diagnostic reports represented eligibility even for a zero-weight
source, whereas the new accumulation must exclude that source. Intersect
the diagnostic mask with positive weights before counting sources or
adding increments. Record the failed unit log, repeat all analytic checks,
and freeze the corrected implementation before tracking. No outcome or
parameter selection is involved in this correction.

The next check found a unit-fixture mismatch: its reference no-age call
used the library's default support-renormalized absence policy, while the
candidate and production runner use the specified FoV-aware censoring.
Set the same explicit policy in the fixture. The candidate formula did not
change for this fixture correction, and no tracking had run yet.
