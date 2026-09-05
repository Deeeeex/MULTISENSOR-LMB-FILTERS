# V279: where the current set error comes from

Post-hoc analysis of the saved paired 160-step, seed-1301 results. No filter was rerun and no policy or selection gate was changed. Scene inputs were regenerated only to read the true cardinality at each time.

| Scale | Arm | E-OSPA | Finite RMSE cells | Same-cell RMSE / fixed | Same-cell RMSE gain | Count share of squared OSPA | Perfect-localization E-OSPA floor |
|:--|:--|--:|--:|:--|--:|:--|:--|
| N=24 | Fixed tree | 125.478 | 99.870% | 22.640 / 22.640 | +0.000% | 96.922--98.736% | 123.557--124.717 m |
| N=24 | Full causal repair | 122.380 | 99.870% | 14.081 / 22.640 | +37.805% | 99.381--99.381% | 121.994--121.994 m |
| N=24 | Sparse causal repair | 122.462 | 99.870% | 12.183 / 22.640 | +46.190% | 99.567--99.567% | 122.184--122.184 m |
| N=36 | Fixed tree | 132.680 | 100.000% | 36.925 / 36.925 | +0.000% | 94.534--97.963% | 128.887--131.355 m |
| N=36 | Full causal repair | 131.795 | 97.101% | 19.586 / 30.654 | +36.107% | 98.868--99.247% | 131.043--131.305 m |
| N=36 | Sparse causal repair | 132.192 | 97.465% | 19.329 / 31.394 | +38.431% | 98.074--99.337% | 130.880--131.761 m |

## Count reconstruction and boundaries

For true count T, absolute count error d and candidate estimate count n in {T-d,T+d}, squared OSPA is c^2*d/max(T,n) plus a nonnegative localization term. A feasible upper bound for that term is min(T,n)/max(T,n) * min(RMSE^2,c^2), using the stored RMSE matching. This bound does not require the two Hungarian assignments to coincide. We eliminate count possibilities inconsistent with these bounds, and retain an interval whenever both are possible. NaN RMSE and nonzero true count identify an empty estimate under the recorded metric definition.

| Scale | Arm | Identified below / equal / above true count | Ambiguous | Total sensor-time cells | Maximum mean E-OSPA reduction from localization alone |
|:--|:--|:--|--:|--:|--:|
| N=24 | Fixed tree | 3729 / 0 / 0 | 111 | 3840 | 1.921 m |
| N=24 | Full causal repair | 3840 / 0 / 0 | 0 | 3840 | 0.386 m |
| N=24 | Sparse causal repair | 3840 / 0 / 0 | 0 | 3840 | 0.279 m |
| N=36 | Fixed tree | 5394 / 0 / 0 | 366 | 5760 | 3.793 m |
| N=36 | Full causal repair | 5724 / 0 / 0 | 36 | 5760 | 0.752 m |
| N=36 | Sparse causal repair | 5639 / 0 / 0 | 121 | 5760 | 1.311 m |

The count share is a ratio of summed squared errors, not a percentage decomposition of mean E-OSPA. The perfect-localization floor is a hypothetical bound with each cell's cardinality unchanged, not an achievable routing result. Same-cell RMSE restricts both arms to cells with finite RMSE; it does not guarantee the same target identities or remove all matching-selection effects. This is one-seed development evidence and an algebraic readout, not causal attribution to a specific edge.

## Sources

- `/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v248-temporal-coupled-routing/RUN/GA/dynamic_topology/evidence/tracking_aligned_v248/m24_temporal_task_coupled_formation_braid_seed1301/minimum_causal_backbone/CAUSAL_MINIMUM_FORMATION_BACKBONE_V242_FULL_EPISODE.mat`; source commit `78735d23f8c85d350ad80e5ddb6d44bb2736d310`.
- `/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v274-x36-minimum-backbone/RUN/GA/dynamic_topology/evidence/tracking_aligned_v274/x36_minimum_backbone_seed1301/minimum_causal_backbone/CAUSAL_MINIMUM_FORMATION_BACKBONE_V242_FULL_EPISODE.mat`; source commit `c9c589d72209fcc8903b338fe77fd9fc26001409`.

Metric definitions: `common/computePositionEuclideanOspa.m`, `common/computeSetRmseOverTime.m`, and `common/summarizeRepeatedMultiGatewayFullEpisodeArm.m`.
