# Method iteration: recency on local evidence increments

Registered 2026-09-08 before any output of the new candidate. The nine
V2V4Real evaluation sequences have already informed the diagnosis and are
now development data for this iteration. Keep all earlier methods, source
manifests and results unchanged. No detector, crop, birth, motion, matching,
pruning, extraction, radio, or age-kernel parameter may change here.

## Hypothesis and first candidate

ER weights a complete posterior's existence log odds by time since direct
geometric sensing opportunity. That posterior includes inherited information
and prior existence as well as the present measurement update. Recency can
therefore amplify an inherited positive claim or remove an inherited
negative claim even when the fresh evidence does not justify that change.
This is a hypothesis to diagnose, not an established cause of the recorded
OSPA gap.

For each local predicted Bernoulli retain the scalar
`delta = logit(r_updated) - logit(r_predicted)` from the current local LMB
update, with the same 1e-9 probability clipping as fusion. This is an
effective local log-odds increment after approximate LMB association, not
an independent calibrated likelihood ratio. Recompute it each scan; do not
relay an old increment as a new local measurement. Transmit one additional
float64 scalar per Bernoulli and check a lossless packet round trip.

Let b be ordinary topology weights normalized over exactly the same eligible
existence inputs, q the unchanged ER recency weights over those inputs, and
eta the unchanged spatial KLA normalizer. The fixed first candidate is

    logit(r_IR) = sum_j b_j logit(r_j) + log(eta)
                  + sum_j (q_j-b_j) delta_j.

Qualified missing-label censors retain delta=0; their supplied upper bound
and eligibility remain unchanged. Equivalently, represented sources have
their predicted log odds pooled with b and only the current update increment
pooled with q. This candidate is called innovation recency (IR). Spatial
weights remain unchanged. Use the existing rho=0.25, lambda=5 seconds. No
new fitted parameter or threshold is introduced. This is a controlled
approximate fusion rule, not an exact distributed Bayes or de-correlation
algorithm.

Required identities: one eligible input gives identity; q=b gives the
ordinary no-age rule; zero local increments give the ordinary no-age rule
regardless of unequal ages; changing increments changes existence only for
fixed spatial inputs. Check finite/SPD outputs, censoring, untouched-prior
exclusion, packet contents and directed-delivery parity. Instrument the ER
reference on the same implementation and compare its state outputs against
the previously saved original to detect changes caused by instrumentation.

## Diagnosis and evaluation

Use all nine already-seen sequences and both existing radio conditions.
Record per-label counterfactual no-age, original ER and IR existence values
on the same pre-fusion inputs, together with age, effective increment,
existence inputs and spatial overlap. Truth is used only after tracking to
score outputs and inspect errors. Do not feed diagnostic truth assignments
to the fusion rule. Report positive and negative shifts, extraction changes,
miss/false costs, count errors and OSPA, not just the selected headline.

First run the original ER with instrumentation and IR on sequences 0000--0002
to check implementation and causal diagnostics; this is a development
preflight. Then run the unchanged candidate on all nine sequences and both
conditions. Include failures and every sequence. Reuse original Local,
no-age, MIL-AM and TC saved results. Resample sequences, not frames, for
descriptive paired intervals; these are development comparisons.

If the first candidate fails the mechanism identities, correct the defect
and rerun every affected result. If it fails empirically, retain it and
record a separate dated amendment before evaluating another candidate.
Do not silently choose a favorable version or reinterpret this cohort as
held out. Any later untouched real-data cohort must be fixed in a separate
manifest before looking at its tracking outcomes. Original synthetic
event families and the no-new-target control must also be rerun with the
selected candidate on their unchanged cached inputs before claiming it
as an improvement.

Primary background: the Bernoulli update separates predicted existence and
measurement likelihood (e.g. Zetterqvist et al., arXiv:2606.09573v1, Section
V-B). That identity motivates the separation; the paper does not establish
or evaluate the IR rule proposed here.
