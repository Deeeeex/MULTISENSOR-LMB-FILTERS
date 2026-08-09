# Conditional V54 direction: receiver-safe label-edge fusion

## Why this branch exists

V52 established that a message can reduce one-round disagreement while
damaging cardinality and tracking. The remaining question is therefore not
only which sensor edge should communicate, but which label carried by that
edge should be allowed to influence a particular receiver. This branch is
activated only if V53 fails its pre-registered X36 development gate. It is not
another threshold-tuning continuation of V51--V53.

The intended problem is communication-constrained distributed LMB fusion with
limited and changing FoVs. A sender that has not observed a label should not
automatically contribute negative evidence through a geometric average, while
a sender that has reliable in-FoV negative evidence should still be allowed to
suppress a false track. The action is thus a receiver--sender--label decision,
not a whole-posterior edge switch.

## What is already known and therefore not novel

Several closely related results constrain the contribution we can claim:

- Wang et al., *Signal Processing*, 2018,
  DOI `10.1016/j.sigpro.2018.04.010`, already assign a separate GCI weight to
  each LMB Bernoulli component using information divergence in a centralized
  multiple-FoV system. Per-label weights alone are not a contribution.
- Gao, Battistelli and Chisci, *IEEE TSP*, 2020,
  DOI `10.1109/TSP.2020.3028496`, derive MIL fusion for LMB densities and
  decompose different FoVs into common and exclusive label subspaces. Replacing
  GCI with label-wise MIL alone is not a contribution.
- Shen et al., *IEEE TSP*, 2022,
  DOI `10.1109/TSP.2022.3154227`, prove that the KLD between same-label-space
  LMB densities decomposes into Bernoulli-component discrepancies and use this
  for separated event triggering. Transmitting only changed Bernoulli
  components is not a contribution.
- Li et al., ICASSP 2023, arXiv `2209.08857`, use a transformer to fuse local
  multi-trajectory densities from sensors with different FoVs. A fully learned
  density-fusion network is therefore neither necessary nor novel here.
- Yi and Chai, *IEEE TSP*, 2021,
  DOI `10.1109/TSP.2021.3087033`, show that one scalar confidence weight is
  inadequate when information quality varies across space and derive
  sub-density-specific heterogeneous fusion. Spatially varying confidence by
  itself is not a contribution.

## Candidate contribution

The defensible gap is a **receiver-conditioned, label-edge fusion policy under
a dynamic communication budget**. It combines four elements that the above
methods do not provide together:

1. use the receiver's current LMB and its observation support to distinguish
   reliable negative evidence from a sender's lack of observation;
2. construct a receiver-safe per-label reference that includes positive
   evidence and credible in-FoV negative evidence, but excludes unsupported
   absence-of-evidence inputs;
3. enumerate the small feasible sender subsets for each receiver--label and
   compute their mixture-aware distortion and incremental payload cost;
4. retain the V46 strongly connected dominant backbone as a hard safety
   layer, then select one residual sender-subset option per label under an
   explicit byte budget while charging each activated edge header only once;
5. optionally train a permutation-equivariant GNN to approximate the expensive
   counterfactual utility, while a deterministic projection enforces the byte,
   FoV and temporal-connectivity constraints.

This keeps learning in the value-estimation layer. The GNN does not directly
generate a graph or a fused posterior.

## Theoretical core to establish before experiments

The repository already provides a componentwise powered-GM LMB-KLA
approximation, and V46--V53 activate it for always-heavy communication. It
retains multiple modes and is a valid frozen numerical receiver for paired
experiments, but it truncates component tuples and is not the exact power of
an arbitrary Gaussian mixture. Before V54 experiments, this approximation
must be isolated as the primary implementable reference and calibrated
against a higher-accuracy numerical density-power oracle on controlled
low-dimensional label cases. The projected-Gaussian GA-LMB path remains a
separate legacy baseline; it is not the receiver currently used by V46--V53.

### Initial numerical calibration

`runPoweredGmKlaGridCalibration` directly integrates a controlled
two-dimensional, three-mode label density on a converged tensor grid. The
81-to-161 point refinement changes `log eta`, existence, mean and covariance
by at most `2.5e-10`, with `3.1e-13` boundary mass. Against that oracle:

| Receiver approximation | TV | KL | Mean error | Existence error |
|:--|--:|--:|--:|--:|
| Frozen powered-GM top-3/max-8 | 0.0334 | 0.0059 | 0.0217 | 0.0048 |
| Untruncated powered-GM 3-by-3 | 0.0334 | 0.0059 | 0.0217 | 0.0048 |
| Projected single Gaussian | 0.5415 | 0.8399 | 0.0620 | 0.0081 |

In this aligned multimodal case, dropping the ninth tuple is negligible; the
remaining powered-GM error comes mainly from the componentwise density-power
approximation. The projected Gaussian has much larger shape distortion even
though moment covariance alone can look competitive. This is a controlled
calibration, not a general error bound or tracking result.

With a correct reference, the paper should target three statements.

### 1. Label-wise distortion decomposition

For reference and selected Bernoulli components
`B_*^ell=(r_*^ell,p_*^ell)` and `B_A^ell=(r_A^ell,p_A^ell)`, use the
reference-to-selected direction

`d_ell(A) = (1-r_*) log((1-r_*)/(1-r_A))`

`          + r_* log(r_*/r_A) + r_* D_KL(p_* || p_A)`.

For matched-label-space LMB densities,
`D_KL(pi_* || pi_A) = sum_ell d_ell(A)`. The direction is intentional: a
candidate that deletes a reference-supported label receives infinite
distortion in the ideal formula instead of being rewarded for becoming
confidently empty. The spatial term must retain the Gaussian-mixture density.
Any theorem stated for exact densities must be separated from the error
introduced by the powered-GM approximation and tuple truncation.

The additive statement is across labels, not across sender edges. Multiple
sources fused into the same label interact through the KLA normalization and
must be evaluated jointly.

At runtime, `buildReceiverSafeLabelSubsetOptions` enumerates the admissible
sender subsets, runs the installed powered-GM receiver for every subset, and
uses `approximateLmbSpatialKldCubature` for the spatial term. The latter is a
positive-weight deterministic cubature approximation of
`D_KL(p_* || p_A)`, not an exact arbitrary-GM KLD. The analytic LMB
decomposition remains exact for exact spatial KLD values; experiments and
claims must disclose the runtime spatial approximation separately.

### 2. Unobserved-label suppression rule

For geometric fusion, adding a sender with low label existence or low density
overlap can reduce the receiver's fused existence probability. If the sender's
FoV and update history show that the label was not observable, this decrease
is absence of evidence rather than reliable negative evidence. The
receiver-safe reference therefore includes a sender for label `ell` only when
the sender supplies positive support or has high predicted observation
opportunity and a recent credible missed-detection update. It excludes a low
existence value caused only by missing FoV support.

The first implementable observable is
`computeLmbLabelObservationOpportunity`: it integrates the actual runtime
`evaluateSensorQuality` model over the predicted label Gaussian mixture with
positive-weight position cubature. It returns expected detection probability
and in-FoV probability without target truth or future measurements. This
continuous opportunity score should enter the evidence rule; a binary check
at the posterior mean is insufficient near FoV and range boundaries.
It must be computed from each local predicted label immediately before the
sensor update and stored as policy metadata. Reconstructing it from an
already fused posterior would mix other sensors' support into the sender's
own observation opportunity.

`updateLmbWithSensorMeasurementAndEvidence` defines this V54-only runtime
boundary without changing the frozen V46--V53 path. It computes opportunity
before the local update and returns separate current-step label evidence.
For an empty measurement set it also resets the association diagnostics
inherited by the legacy missed-detection update; otherwise a detection from a
previous step could be misread as current positive support.

`classifyLmbLabelLocalEvidence` then combines that opportunity with the
label-specific detection-association mass and the predicted-to-updated
existence change. It assigns one of four observable states: positive support,
credible negative evidence, unsupported absence, or ambiguous evidence. Only
positive support and credible negative evidence are admissible to the safe
reference; the receiver's own posterior remains present independently. The
initial semantic thresholds are explicit configuration values and must be
frozen before held-out validation rather than retuned per scene or scale.

This safe reference, not conventional all-source fusion, is the optimization
target. Conventional full fusion remains a baseline. In addition to the KLD
distortion, every admissible option directly enforces a supported-label
existence-retention floor; the paper should not infer that floor from a loose
divergence inequality.

### 3. Budgeted projection guarantee

For a fixed physical page and bounded incoming formation degree, enumerate
the feasible sender subsets for each receiver--label. Each subset is one
option with a receiver-approximation distortion, incremental label bytes,
retention status and residual formation-edge activation mask. Distortion is
additive across labels, but wire cost is not a plain multiple-choice knapsack:
labels sharing one sender edge also share its packet header. The exact dynamic
program therefore uses `(label stage, active residual-edge mask)` as its state
and Pareto-prunes cost/distortion states only within the same edge mask.
`selectReceiverSafeLabelOptionsExact` implements this projection for the
enumerated option table.

The frozen V46 dominant cycle remains present at every step, so temporal
strong connectivity is a construction invariant rather than a learned or
post-hoc constraint. V54 optimizes label payloads on residual cross-formation
edges; a per-edge greedy ranking remains only an approximate ablation.

The final statement separates two guarantees: the posterior distortion and
existence-retention bounds are label-wise, while reachability is a graph-level
temporal guarantee.

## Data-driven extension

The exact selector supplies teacher targets consisting of per-label subset
value, edge marginal values, retention risk and the selected packet mask. A
GNN can use only runtime-observable features: existence probability, mixture overlap,
innovation/update age, FoV support, sender/receiver covariance, link
reliability and recent formation-level traffic. Training truth may be used in
the loss only for an explicitly separate oracle/teacher experiment; the
deployable model cannot read target truth or future measurements.

The learned arm is accepted only if it reproduces the analytic action on held
out scene families and preserves the deterministic projection guarantees. A
small tracking gain obtained by violating the byte budget or using scene truth
does not count.

## Required baselines

Any experiment for this branch must include:

- the frozen componentwise powered-GM LMB-KLA approximation with fixed scalar
  weights and disclosed tuple/component limits;
- a higher-accuracy numerical density-power oracle on controlled label cases,
  used to calibrate approximation error rather than as a full tracker;
- projected-Gaussian GA-LMB, clearly marked as the legacy engineering
  receiver;
- fixed per-Bernoulli information weights in the spirit of Wang et al.;
- separated event-triggered LMB communication;
- label-subspace MIL or the closest tractable implementation for different
  FoVs;
- analytic receiver-safe label selection;
- learned utility approximation, if training is pursued.

The primary comparison is not whether label selection sends fewer bytes than
full fusion; it is whether it improves tracking and cardinality at the same or
lower byte budget across both M24 and X36 and across radial, convoy,
merge-split and curved-corridor scene families.
