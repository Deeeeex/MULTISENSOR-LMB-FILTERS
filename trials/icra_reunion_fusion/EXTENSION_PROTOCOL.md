# Qualification and existence-age ablation v2

2026-09-08, registered after the completed v1 screen and before either new
arm is run. Reuse all nine v1 input caches; do not generate new observations,
change paths/FoV/births, alter source labels, or search decay parameters.

## Evidence motivating the change

Independent exhaustive assignment recomputation verified 51,840 node-frames.
On split/churn, the existing lineage baseline's mean OSPA is 2.661/0.770 m,
versus fov 3.362/2.276 m. It has only 0.175 m^2 mean false cost in the no-new
control, but that is seven false estimates in 2,880 node-frames versus fov's
zero, so the prespecified false-positive screen fails. After the departure,
lineage's false cost is 17.1 m^2 versus fov's 14.5 and recent's 12.8.

The recent arm passes the v1 descriptive screen but loses to lineage on both
event-scene mean OSPA. Its common-target RMSE also increases versus fov.
Thus this is a fusion tradeoff, not a successful novel method. The additional
question is whether temporal trust must change spatial pooling at all.

## Two arms; no new numeric parameters

- `lineage_recent`: existing observation-lineage eligibility plus the v1
  direct-opportunity multiplier, applied to spatial and existence weights.
  This is a composition control, not the preferred method by construction.
- `qualified_exist`: the same eligibility and existence weights, but ordinary
  eligible-input Metropolis weights for spatial fusion. This preserves the
  spatial-pooling rule of the lineage baseline. The same powered-GM spatial
  normalizer still contributes to existence pooling, so this is **not**
  arithmetic-existence MIL and not exact single-objective KLA when the two
  weight vectors differ. No universal consistency or calibration is claimed.

Both use the already frozen factor `0.25+0.75 exp(-age_seconds/5)` and the
existing observable-absence censor. Low existence after a valid recent
missed detection counts as negative evidence; no truth or high-r preference
qualifies inputs. Local direct timestamps are not refreshed by relaying.
There are no new bytes beyond the already matched metadata contract.

## Decisions and limits

Report all v1 metrics and paired common-truth assignment RMSE. Relative to
lineage, a balanced mechanism signal requires: no more than 2% mean OSPA
regression on either event scene; at least 10% lower post-departure false
cost; at most 2% pooled common-target RMSE regression on either event scene;
and no-new false cost no higher than fov's zero. Also compare both arms to
recent; its position/existence tradeoff must remain visible.

This is a development-set ablation. A pass would justify a short independent
seed check, not an ICRA readiness or novelty claim. Failure stops this
combination; do not change the floor, half-life, output threshold or truth
evaluation to obtain a pass. Existing work on information-weighted and
multi-view LMB remains mandatory prior art; the local prototypes do not
constitute faithful reproductions of those complete published methods.

The extension runner is mechanically derived from the frozen v1 harness,
changing only the entry point, fusion configuration and wrapper call. The
v1 sources, results and hash manifest remain untouched. Every extension run
loads the exact saved model, truth, measurements, uniforms and adjacency.
