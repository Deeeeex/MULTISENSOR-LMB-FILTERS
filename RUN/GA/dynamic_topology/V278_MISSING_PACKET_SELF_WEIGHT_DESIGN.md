# One controlled step toward delivery-aware sparse causal fusion

Freeze the V242 topology and powered-GM fusion settings. On the opened X36
temporal-coupled formation-braid seed 1301, change only
`missingNeighborWeightMode` from the inherited `renormalize` default to the
existing `self` option. Reuse the completed V274 V242 and fixed-tree results.
This run is a receiver-side ablation within the mainline, not a new topology
family, GNN or payload-compression claim.

For planned receiver weights w and missing mass m, renormalization makes each
surviving weight w_j/(1-m). Self fallback keeps surviving neighbor weights w_j
and increases self weight by m. The latter prevents accidental amplification
of a surviving weak neighbor when the dominant input is absent. It can also
slow useful propagation, so no tracking improvement is assumed.

Both rules have row L1 distance 2m from the planned weights; self fallback does
not minimize this norm more than renormalization. Its distinct property is
where the missing mass goes. Neither rule ensures double stochasticity. Any
claim about label existence must also account for the density-overlap term in
LMB-KLA and the existing label-specific absence rules.

The full 160-step comparison reports E-OSPA, conditional set RMSE, mean absolute
cardinality error, focus consistency, attempted bytes and the weakest formation.
One frozen, low-cost follow-up gate precedes an M24 run: E-OSPA strictly improves
over V242, consistency and cardinality do not worsen, RMSE loses at most 1%,
the weakest formation E gain is at least -1%, and at least 5% attempted-byte
saving over fixed routing remains. No threshold is changed after the result.
A failed result closes this handling rule as a standalone fix; it does not
justify a parameter sweep. A passed result permits M24 and then independent
scene/seed validation, not an immediate paper-level success claim.

Run from a clean source checkout:

```sh
octave --no-gui --quiet --eval "addpath(genpath(pwd)); runMissingPacketSelfWeightV278FullEpisode(struct('referenceResultPath','/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v274-x36-minimum-backbone/RUN/GA/dynamic_topology/evidence/tracking_aligned_v274/x36_minimum_backbone_seed1301/minimum_causal_backbone/CAUSAL_MINIMUM_FORMATION_BACKBONE_V242_FULL_EPISODE.mat'));" 2>&1 | tee RUN/GA/dynamic_topology/evidence/tracking_aligned_v278/v278_full_episode.log
```

The execution context remains the V242 topology context; the result explicitly
names V278 and stores the changed receiver rule and reference source commit.
No authorization or simulator gate is disabled.
