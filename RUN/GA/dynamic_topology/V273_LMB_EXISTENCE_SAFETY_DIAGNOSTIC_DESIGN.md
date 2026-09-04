# V273 LMB existence-safety diagnostic

## Question

V272 showed that a route can improve temporal receiver coverage and finite-horizon
mixing while worsening every formation's E-OSPA.  A lower conditional set RMSE
did not rescue the result because the same action increased cardinality error.
The next question is therefore narrower and falsifiable:

> Can the current LMB posteriors, without target truth or future measurements,
> identify gateway substitutions that will suppress label existence under the
> executed KLA operator?

V273 is a diagnostic, not a new tracking arm.  It reuses the completed V250 H=3
counterfactual bank at anchors 70, 84 and 151.  It does not open another scene,
seed, time window, or X36 outcome.

## Operator-aligned predictor

For every V250 candidate, V273 reconstructs the V242 reference route and the
candidate route from the same current physical page.  It then enumerates the
independent delivery outcomes of each receiver's selected incoming links and
executes `fuseLmbPosteriorsByLabel` with the same mixture-aware configuration as
the recursive tracker.  Thus, the predicted label existence already includes
the powered-GM spatial normalizer, missing-message weight normalization, and
current link reliability.

The candidate is compared with the reference through asymmetric quantities:

- reference-weighted existence shortfall;
- expected-cardinality loss by receiver and formation;
- minimum retention of a reference-supported label;
- count and mass of labels predicted to cross from MAP-positive to MAP-negative;
- expected one-round posterior-disagreement change.

The first four quantities preserve dangerous lower-tail labels instead of
averaging them into a single pairwise compatibility score.  The disagreement
term is retained only to test whether contraction and set safety are aligned; it
is not treated as a recursive tracking certificate.

## Outcome comparison

Truth is used only through the already completed V250 H=3 outcomes.  For each
nonreference candidate V273 records:

- network E-OSPA gain;
- minimum-formation E-OSPA gain;
- change in mean absolute cardinality error;
- conditional set-RMSE and consistency gains as secondary diagnostics.

The primary target is existence-related harm: increased cardinality error or a
material E-OSPA regression.  Conditional RMSE improvement is never interpreted
as localization improvement when cardinality or E-OSPA worsens.

## Generalization test and stopping rule

V273 reports pooled and per-anchor rank association, then performs
leave-one-anchor-out threshold selection for each monotone safety quantity.  A
signal is considered reusable only if its direction is consistent at all three
anchors and the threshold learned from the other two anchors rejects material
existence harm on the held-out anchor without eliminating every useful action.

- If no quantity passes, close contraction-first gateway substitution as a
  deployable method family.  Do not train a GNN on these three anchors.
- If a quantity passes, the next policy may use it only as a hard safety
  constraint.  Structural mixing may rank candidates inside the safe set.
- Any online version must expose and charge the posterior synopsis/control
  metadata needed to evaluate the constraint; this centralized diagnostic is
  not itself deployment evidence.

## Evidence boundary

V273 is post-hoc development evidence from one M24 seed and three short windows.
It cannot authorize a full episode, X36, multistyle validation, a learned model,
or a paper claim.  Failed results belong only in the experiment record.  The
main progress document remains unchanged unless the diagnostic establishes a
stable method decision.
