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
2. compute a mixture-aware single-label serve/hold counterfactual for each
   feasible receiver--sender--label packet;
3. select label packets under an explicit byte budget and a temporal
   information-flow constraint;
4. optionally train a permutation-equivariant GNN to approximate the expensive
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

For LMB densities on a matched label space, the divergence between a selected
fusion result and the frozen full-service reference can be decomposed into a sum of
Bernoulli terms. This yields an additive label-packet distortion measure with
an existence term and a single-target-density term. The derivation must retain
the Gaussian-mixture density rather than collapsing every track to one
Gaussian before the comparison. Any theorem stated for exact densities must
be separated from the error introduced by the powered-GM approximation and
tuple truncation.

### 2. Unobserved-label suppression rule

For geometric fusion, adding a sender with low label existence or low density
overlap can reduce the receiver's fused existence probability. If the sender's
FoV and update history show that the label was not observable, this decrease
is absence of evidence rather than reliable negative evidence. The method
censors that label contribution and renormalizes the remaining label-specific
weights. The theorem should bound the full-reference distortion and prove that
the censoring action cannot introduce a supported-label existence decrease
larger than the registered tolerance.

### 3. Budgeted projection guarantee

For a fixed physical page, additive label distortion turns packet selection
into a budgeted allocation problem. The analytic selector ranks exact marginal
utilities and then projects the selected packets through per-receiver byte
limits and the previous/current pulse information-flow constraint. The final
statement must separate two guarantees: the posterior distortion bound is
label-wise, while reachability is a graph-level temporal guarantee.

## Data-driven extension

The exact selector supplies teacher targets consisting of per-label marginal
utility, retention risk and the selected packet mask. A GNN can use only
runtime-observable features: existence probability, mixture overlap,
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
