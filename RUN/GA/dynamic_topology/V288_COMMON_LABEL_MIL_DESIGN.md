# V288: isolate the shared-label LMB-MIL fusion rule

2026-09-06. Development experiment; design fixed before the candidate run.

## Question and method decision

Does replacing the current geometric receiver by a literature-based arithmetic
LMB receiver improve joint set recovery and localization under the **same V242
route**? This is a fusion-family control, not a new routing method, a label
matching implementation, or a diagnosis that the existing shared labels are wrong.

The accessible author version of Gao, Battistelli and Chisci,
[Fusion of labeled RFS densities with minimum information loss](https://arxiv.org/html/1911.01083v1),
Proposition 3, equations (26)--(28), gives the LMB-constrained minimizer of
`sum_i omega_i KL(pi_i || pi)`:

`r_l = sum_i omega_i r_il`,
`p_l(x) = sum_i omega_i r_il p_il(x) / r_l`.

The same normalized source weights must be used in both expressions. Concatenating
the input GM components implements the spatial rule exactly before reduction.
There is no density-overlap multiplier in the existence update. This is **MIL,
not KLA**; changing the KL direction is already established work.

## Representation and implementation boundary

- Preserve birth-time/birth-location labels, local filtering, measurements,
  extraction, sensing schedule, geometry, delivery draws and the V242 route.
- Lift the represented input label sets to their union. An absent/pruned label
  has `r=0`; its source weight is not removed from the denominator. A label
  exclusive to one source is attenuated by that source's weight, not vetoed.
- This is MIL of the represented posteriors, **not** the complete different-FoV
  partition/matching algorithm of the paper. Payload pruning remains an
  approximation to the untransmitted posterior. No censored pseudo-density or
  unobservable-source exclusion is silently transplanted from KLA.
- Keep all input spatial modes before the existing eight-component output cap.
  Coalesce exactly identical kernels without changing the density, then
  retain the highest-weight components and normalize; record the removed
  conditional mass. Thus the implemented recursive receiver is reduced-GM MIL,
  not exact unrestricted-density MIL.
- Do not stack V284 prior exclusion, geometric relabeling, new weights, a GNN,
  additional packets, or a parameter sweep. Refuse unequal spatial/existence
  weights and KLA-specific per-label overrides in this isolated baseline.

The existing `aaLmbTrackMerging` requires equal object ordering at every input
and therefore is not a suitable sparse-message receiver. Its shared-weight,
existence-weighted GM formula is reused conceptually, not its indexing.

## Paired protocol and stop rule

X36 formation-braid, seed 1301, steps 1--40 of the original 160-step scene.
Generate the full original observations before cropping. Reuse the saved V282
V242 trace; run only the new arm with the same filter seed offset. First perform
small formula checks and a two-step integration run. No repeated baseline run.

Report E-OSPA, absolute count error, conditional matched RMSE with its finite
coverage, common-finite-cell RMSE, representative disagreement over this entire
prefix, per-node/per-formation tails, attempted/delivered messages and bytes,
elapsed time, and GM truncation mass. Conditional RMSE still has a changing
matched set and must not be interpreted alone. Prefix disagreement is not the
paper's full-episode focus-window statistic.

Proceed to a same-fusion fixed-route comparison only if the 40-step candidate
improves E-OSPA by at least 1%, lowers count error, does not increase representative
disagreement, has at most 1% common-finite RMSE degradation, has at most 1%
worst-formation E-OSPA degradation, and uses no more than 1.05 times the reference
attempted bytes. Attempted/delivered route masks must remain identical; otherwise
report that confound before any attribution. This is a development screen, not
the final joint-goal threshold.

On a failed screen, keep the result in experiment records, do not promote it to
the main best-method table, and do not tune source weights or truncation for this
opened seed. Matching and FoV-aware MIL remain separate hypotheses; a failure
of this control cannot reject the complete published method.

## Why matching is not switched on at the same time

Section V-B of the same author version explicitly allows unmatched tracks and
uses spatial-density divergence with a stated unmatched cost. It also identifies
limitations of forced matching and existence-dominated matching costs. That
provides a usable basis for a later comparison, but not evidence that our shared
birth semantics should first be overwritten. Isolating MIL avoids attributing
a change of fusion family to label matching or to routing.
