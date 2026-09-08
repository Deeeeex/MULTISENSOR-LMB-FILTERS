# Fair marked-likelihood controls for evidence-qualified recency

Registered 2026-09-08 while ECR completes its original unmarked-local-filter
development replay, before any marked-local-filter tracking outcome. The
new information available to ECR-S/C must also be offered to a principled
no-age baseline. All nine validation sequences remain seen development.
No outcome from the new 25-sequence fusion-selection cohort is available.

## Fixed observation mark model

Use exactly the previously frozen sequence-held-out logistic calibrators
and the same per-sensor 12 m matching labels. Do not refit their coefficients,
penalty or clipping. In each training fold compute the sequence-balanced
positive fraction pi using the same weights as calibration. For calibrated
objectness p(s), define the mark likelihood ratio

    ell(s) = [p(s)/(1-p(s))] / [pi/(1-pi)].

This is the Bayes-odds identity for f_target(s)/f_clutter(s). A mark model
independent of location given the detection class multiplies each detection
association column by ell(s); the missed-detection branch is unchanged.
It is an approximate empirical marked measurement model: the 2D assignment
label and regularized calibration are not exact generative class labels,
and conditional independence, score stationarity and clutter-rate accuracy
are assumptions. p_D remains the existing fixed nominal value. Introducing
measurement amplitudes/marks into LMB associations is established prior
work, e.g. https://doi.org/10.1049/iet-rsn.2018.5293 ; no novelty claim is
attached to this control itself.

The trial-local adapter multiplies the already generated detection terms,
leaving conditional Gaussian parameters, birth selection, measurement
coordinates/sets, dynamics and ordinary clutter intensity unchanged.
For mixtures, a mark that is constant across components cancels inside each
measurement-conditioned mixture normalization. Apply it once in association
weights only, not again in normalized posterior component weights.

All arms share these exact marked local updates and calibration folds:
marked No-age, marked ER, and marked ECR-A/S/C. ECR constraints and metadata
are reused byte-for-byte from the prior freeze. This five-arm comparison
separates mark-model benefits from age weights and the positive-age ceiling.
No new tracking parameter, selected score threshold or fitted fusion weight.

Analytic preflight must prove unity likelihood ratios reproduce the original
update and diagnostics exactly; one Bernoulli and one observation reproduce
the scalar missed-plus-detected Bayes existence; greater mark ratio increases
existence in that fixed scalar example; W exposes the actual marked update;
empty measurements still use the original missed-detection update. Run the
full nine sequences and both radio conditions, independently re-evaluate
all output metrics and existence equations, and include the unmarked controls
when reporting the contribution of the mark model.

Only after this full comparison, choose and freeze a final method for the
25 unused fusion-selection sequences. Include the corresponding marked
No-age/ER controls in that held-out comparison if the final method uses the
marked update. Preserve all adverse development arms and unchanged original
ER/no-age results. No paper-facing main-result claim precedes that evidence.
