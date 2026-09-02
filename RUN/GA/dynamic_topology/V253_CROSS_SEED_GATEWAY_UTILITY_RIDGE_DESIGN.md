# V253 cross-seed gateway utility ridge

## Why the V251 teacher objective is insufficient

V251 ranks the action that helps the formation with the largest realized
reference RMSE.  That label is useful for diagnosing finite-horizon headroom,
but its identity is not reliably observable: posterior covariance and
within-formation disagreement matched the truth-worst formation in only 0/3
and 1/3 opened windows.  A selector trained only to imitate that privileged
teacher can therefore fit three anchors yet have no causal rule for deciding
which formation is actually worst at deployment.

V253 removes the privileged formation identity from the deployment objective.
For every candidate it defines a scalar robust utility

`u = min(g_E, g_R, g_C, g_B, g_FE + 2, g_FR + 2)`,

where the first four terms are percentage gains in network E-OSPA, RMSE,
consistency and attempted bytes, and the last two are the weakest-formation
E-OSPA/RMSE gains with the frozen 2% tolerance expressed as slack.  The V242
reference has utility zero.  Positive utility therefore means that every core
metric improves, bytes do not increase and neither formation tail crosses the
allowed regression boundary.

## Causal features and reference fallback

The regression input is the candidate-minus-reference feature vector.  The
zero vector maps to zero without an intercept, so the model's output is an
estimated robust advantage over doing nothing.  The selector chooses the
highest predicted utility only when it exceeds a frozen activation threshold;
otherwise it uses the ordinary V242 gateway assignment.  This gives ridge a
real abstention action instead of forcing a potentially harmful non-reference
candidate at every page.

Two representations are compared using training seeds only.  `mean-47` is the
V251 permutation-invariant summary.  `distribution-167` adds standard
deviation, minimum and maximum pooling over the six directed gateway edges and
over changed-edge deltas.  The added statistics preserve cross-formation
heterogeneity and receiver-need tails without sensor IDs, formation IDs, truth
or future data.

## Frozen selection and holdout gate

Seeds 1302 and 1303 select the feature set, ridge lambda and activation
threshold by leave-one-seed-out evaluation.  The deterministic selection key
prioritizes the number of seed folds passing the joint metric/tail gate, then
the fraction of realized safe-positive actions, the worst fold's joint score,
aggregate robust utility and finally the smaller representation.

The chosen configuration is retrained on both training seeds and evaluated
once on seed 1304.  It passes only if at least three of the six held-out
windows choose a realized positive-utility action, aggregate E-OSPA, RMSE,
consistency and bytes all improve, and the weakest formation remains within
the 2% E-OSPA/RMSE tolerance.  A pass authorizes a frozen complete-episode M24
policy on seed 1305.  A failure sends the method back to representation design
or stops this learning route; it does not justify a GNN merely because ridge
failed.
