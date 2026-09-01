# V235 dynamic-routing tail localization

## Result that changes the method

The corrected V227 M24 pair establishes real headroom for dynamic routing.
Relative to the fixed all-time-physical tree, the current-physical tree lowers
full-episode E-OSPA by 5.845%, position RMSE by 7.185%, focus/terminal
consistency error by 5.122%/4.960%, and attempted bytes by 7.603%.  The worst
sensor also improves on both tracking metrics.  Dynamic topology is therefore
not merely a communication heuristic; it can materially improve the recursive
tracking state under an exactly matched message and weight budget.

The remaining failure is localized rather than global.  F1--F3 improve on
both full-episode E-OSPA and RMSE, while F4 degrades by 1.306% and 16.185%.
Reconstructing the five causal route changes gives the following sequence:

- through t=56, F4 uses S18->S20 and S2->S24 and is slightly positive over
  the static reference;
- at t=57, S18->S20 is replaced by S13->S20, after which the F4 segment turns
  negative on both E-OSPA and RMSE;
- at t=70, the complete F4 input rows already equal the static route
  S2->S19 and S13->S20, yet the long t=70--152 segment remains 5.737% worse
  in E-OSPA and 27.084% worse in RMSE.

The last point rules out a purely instantaneous explanation.  A route switch
changes the posterior recursively; restoring the same current edge set does
not restore the static posterior state.  The relevant risk is a persistent
source-substitution debt, not simply whether the current physical tree is
connected or reliable.

## Method implication

The next controller should keep the geometry/reliability tree as a proposal,
then apply a semantic-continuity shield before accepting changed incoming
sources for a formation.  The shield uses only causal compact LMB summaries:
label support overlap, existence disagreement, position compatibility,
uncertainty/evidence changes, recent source substitutions and accumulated
dwell/recovery state.  It may choose the proposed dynamic rows, retain the
incumbent rows, or fall back to the registered static rows.  Connectivity,
two-input message parity and the KLA weight multiset remain deterministic.

An H=3 value model can rank the safe alternatives, but it cannot override the
physical, connectivity or semantic-continuity projection.  A ridge baseline
must be compared before a formation GNN; the GNN is retained only if grouped
unseen-trajectory and closed-loop results are better.  Offline truth may label
source substitutions, while runtime features remain posterior/graph/history
only.

The first mechanism probe should replace F4 by its static rows from the t=57
transition onward while leaving all other formations dynamic.  This schedule
is outcome-derived and therefore only a teacher upper bound.  Its purpose is
to determine whether formation-local shielding can preserve the large network
mean gains and close the F4 tail before implementing a causal guard.
