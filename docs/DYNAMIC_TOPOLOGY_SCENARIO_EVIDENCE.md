# Evidence package: multi-formation dynamic-topology scenario

## Question

What experiment hierarchy should replace the legacy 4+4 setup before designing a learned dynamic-topology method, and what evidence gate should decide whether the direction is worth pursuing?

The supported decision is whether to adopt D12/M24/X36 as the scenario ladder and proceed first to a topology-only oracle-gap implementation.

## Scope

Included:

- repository ancestry from `282ca8180510315424dbb488ce7cfd80e624115f`
  through the current implementation worktree;
- the legacy 4+4 experiment driver and current topology selector;
- the existing N50 topology results and cross-layer smoke results;
- the current projected-Gaussian GA-LMB fusion implementation;
- representative consensus/event-triggered LMB papers and a primary learned-communication paper;
- arithmetic and waypoint feasibility checks for the proposed scenario;
- the configurable R8/D12/M24/X36 generators, fail-closed topology handling,
  attempted-byte accounting, mixture-aware reference configuration and D12
  paired runner;
- deterministic regression tests and one 8-step software/runtime smoke.
- one bounded, full-window, three-seed D12 paired screen and its independent
  post-processing audit.
- difficulty-gated D12/M24/X36 hard presets, a three-seed geometry audit,
  a common safe candidate-pool interface, and counterfactual task-risk
  teacher diagnostics;
- a one-seed D12 closed-loop current-task-risk screen, an in-sample
  shortlisted fixed-graph control, one M24 scale/action-signal snapshot,
  and a conditional six-step M24 strategy screen from a shared posterior
  checkpoint;
- an X36-hard behavior/signal audit, one- and three-step six-arm conditional
  screens, the aggregate-observability-matched X36 diagnostic, and the
  geometry-gated X36-clean-scale behavior, one-step and three-step screens.
- a directed, receiver-specific KLA routing path with custom fusion weights,
  a truth-dependent diagnostic teacher, deployment-observable edge features,
  a receiver-held-out kNN policy, and paired one-step M24/X36 checkpoint
  screens against both static communication and local-only estimation.

Excluded:

- an exhaustive performance sweep over all 48 fixed D12 graphs;
- a multi-seed held-out validation of the current-task-risk teacher;
- multi-seed, full-episode M24/X36 tracking validation and a complete
  runtime-memory scaling curve;
- multi-seed or full-episode X36-clean-scale validation;
- an exact arbitrary-GM density-power implementation;
- seed- and time-held-out training of the learned routing model;
- multi-step or full-episode validation of directed routing;
- a comprehensive systematic review of every distributed LMB paper.

## Risk Tier

**L3.** The old undirected one-step teacher has a confirmed stop finding, and
repeating symmetric KLA makes the M24 checkpoint worse. A new receiver-specific
directed-routing action space produces a large privileged-teacher gap on both
M24 and X36. Its deployment-observable kNN policy passes the one-step mean
tracking and communication screen on M24 and transfers without X36 labels to
an 11.12% gain over static and 6.41% over local at the X36 checkpoint.
However, M24 is the training checkpoint, X36 is only a single-seed/single-time
zero-shot screen, and M24 has a 0.22% strict worst-node caveat versus static.
There is no multi-step, seed-held-out, full-episode, or independent validation.
The current evidence authorizes continued directed-routing validation and a
later GNN replacement study, not a paper claim of scalable superiority.

## Claims

| ID | Claim | Confidence | Evidence IDs | Caveats |
|:--|:--|:--:|:--|:--|
| C1 | The current 4+4 driver is hard-coded for eight sensors and cannot represent the proposed multi-formation scale by changing one parameter. | High | E1, E2 | Core filter functions are more generic than the experiment driver. |
| C2 | The old 4+4 experiment does not provide positive evidence for dynamic topology; its final N50 dynamic arms were dominated or degraded. | High | E3, E4 | This is evidence about the old scene and current heuristic, not a proof that all dynamic topology is ineffective. |
| C3 | A 12/24/36 sensor ladder creates a substantially larger topology decision space while remaining a more defensible first step than jumping to 48+ sensors. | Medium | E5, E6, E7 | One M24 behavior runtime is now measured in C22; the full runtime-memory scaling curve remains unmeasured. |
| C4 | The proposed M24 formation-level ring can have a physically feasible static backbone with a 900 m communication radius along the specified pchip center trajectories. | High | E8 | The final implemented generator must reproduce this check after any parameter change. |
| C5 | The audited baseline topology fallback could violate the intended physical graph, and its bridge helper was specialized to two equal groups. | High | E9 | The new fail-closed handling and scalable safe candidate projection supersede this baseline behavior in C9/C16. |
| C6 | The current GA fusion path projects each Gaussian mixture to one Gaussian before fusion, so it cannot be the paper-level reference for mixture-aware LMB-KLA claims. | High | E10 | A density-level mixture reference will still require a documented numerical approximation. |
| C7 | An exact-oracle gate should precede GNN design because it can falsify the premise that dynamic edge choice has useful residual value. | Medium | E3, E4, E11 | The 10%/5% practical-effect thresholds are proposed preregistration values, not evidence-derived constants. |
| C8 | The current worktree provides one-call R8/D12/M24/X36 scene presets and validates trajectory bounds, separation, edge budgets, group/global connectivity, all-time static physical feasibility, D12 candidate count and handovers. | High | E12, E13 | R8's old runner remains the exact numerical-regression authority; the new R8 preset is an interface regression. |
| C9 | Dynamic topology now fails closed when the physical graph is infeasible, accepts time-varying edge loss, and reports attempted separately from delivered payload bytes. | High | E14, E13 | Control/ACK bytes are not yet added; current accounting is payload-only. |
| C10 | The implemented mixture-aware reference preserves multiple components and passes single-Gaussian and separated-identical-mixture checks, but remains a componentwise powered-GM approximation rather than exact arbitrary-mixture KLA. | High | E15, E13 | Paper-level theory must retain this approximation boundary or replace it with a stronger numerical reference. |
| C11 | The first 8-step smoke is not evidence of a dynamic-topology gain; it exposed metric saturation under the inherited 5 m OSPA cutoff and confirmed that a myopic one-step oracle is not a closed-loop upper bound. | High | E16 | The corrected smoke covers only the pre-handover window and one seed. |
| C12 | In the three-seed handover window, posterior-discrepancy improved focus E-OSPA by 6.77% and posterior disagreement by 10.76% over the geometry-selected static graph, with 3/3 paired directions, at most 1.37% attempted-byte mismatch, and zero topology infeasibility. | High | E17, E19 | N=3 is directional only, and the static graph is not an exhaustive offline performance optimum. |
| C13 | Both one-step diagnostic policies were dominated by posterior-discrepancy on focus tracking and posterior disagreement in all three seeds; they therefore cannot be used as an upper bound or GNN teacher in the current design. | High | E17, E19 | This invalidates the current teacher, not every possible learned topology policy. |
| C14 | The arm called robust-static is selected by a mean/worst geometric-distance score, not by evaluating tracking performance over all 48 fixed candidates. | High | E18 | No paper-level dynamic gain may be claimed until a train-selected, held-out fixed-graph baseline is added. |
| C15 | D12-hard, M24-hard and X36-hard pass explicit low-blackout, handover, overlap, ownership-balance and blockage-pressure gates on seeds 7/17/27. | High | E20, E21 | These are geometry/observability checks, not evidence that the LMB filter is healthy or that dynamic topology improves tracking. |
| C16 | The common candidate-pool interface keeps each formation ring, enforces physical/budget/degree/connectivity constraints, uses all 48 registered D12 graphs, and returns feasible projected pools for M24/X36. | High | E22, E21 | The projected M24/X36 pools are deterministic proposals, not exhaustive optima. |
| C17 | The new task-risk label scores labelled existence, expected state error and covariance after mixture-aware fusion, and is invariant to the configured topology-switch penalty. | High | E23, E21 | This validates label construction, not closed-loop predictive usefulness. |
| C18 | On D12-hard seed 7, open-loop look-ahead through 20 steps retained exactly the same topology as current task risk at t=30/40/60/80; it therefore adds no observed teacher information despite a 2.5%–3.7% candidate-risk spread. | High | E24, E25 | One seed is enough to falsify the claim that this implementation necessarily adds future information, but not to characterize every scene. |
| C19 | Teacher-forced three-step rollouts that consume future measurements increase the candidate-risk spread to 2.07% at t=30 and 4.27% at t=60, but still select the same graph as current task risk even when the candidate graph persists for all three steps. | High | E26, E27 | This is a one-seed action-label diagnostic; C20 provides the first realized comparison for current task risk, but not a multi-seed validation. |
| C20 | In a D12-hard seed-7 closed-loop run through step 60, the pure current task-risk teacher improves focus E-OSPA by 14.42%, focus cardinality error by 27.11%, and focus posterior disagreement by 3.54% over posterior-discrepancy, while using 1.27% fewer attempted bytes and zero infeasible topologies. | High | E28 | This is a privileged teacher and a single screening seed, not a deployable method or paper-level effect estimate. |
| C21 | Registered fixed candidate 16 is stronger than geometry-static on D12-hard seed 7, yet the task-risk teacher still improves focus E-OSPA by 8.64%, focus cardinality error by 16.34%, and focus posterior disagreement by 8.00% at a 1.28% attempted-byte increase. | High | E28, E29 | Candidate 16 was shortlisted from in-sample teacher diagnostics, not selected on independent training seeds; this remains a screening-strength static control. |
| C22 | At the M24-hard seed-7 t=75 snapshot, the scalable pool provides 28 selection-valid actions; current task risk spans 15.37% across them and its best action reduces the surrogate by 2.35% relative to the registered static graph, with 24.70 s teacher scoring after a 559.37 s behavior trajectory. | High | E30 | This establishes scale/action sensitivity at one posterior state only; the selected action was not rolled out, so it is not M24 tracking-gain evidence. |
| C23 | At the same M24-hard seed-7 t=75 posterior, mean, mean-CVaR and CVaR sensor-risk aggregation select the same one-step action; that action reduces realized E-OSPA from 25.5087 to 22.6428 (11.23%) at 0.33% attempted-byte mismatch and zero infeasibility. | High | E31 | One state cannot establish that the aggregation rules are generally equivalent; it only shows no action benefit from tail weighting at this checkpoint. |
| C24 | In the conditional M24-hard seed-7 t=75–80 screen, pure mean current task risk is the best of all six evaluated constraint-eligible arms: focus E-OSPA falls from 23.0892 to 21.1925 (8.21%), worst-node E-OSPA from 47.3908 to 35.9637, MAP-set disagreement from 25.6338 to 23.8603, and cardinality error from 0.8056 to 0.6944, with 29 edges, zero infeasibility, 0.0577% attempted-byte mismatch and boundary-aware churn 0.0227. | High | E32, E33, E34 | This is a privileged teacher, one seed, six conditional steps, and a finite proposal pool. Posterior disagreement worsens from 0.8205 to 0.8446; this is tracking-primary evidence, not a universal consensus improvement or global optimum. |
| C25 | X36-hard is not a clean scale-only transfer from M24-hard: it simultaneously changes 24/16 to 36/24 sensors/targets and degrades half-FoV from 145° to 60°, range from 340 m to 280 m, detection from 0.88 to 0.86, clutter from 4 to 5, birth probability from 0.06 to 0.05, and quality reference range from 300 m to 250 m. | High | E35 | The stress preset remains useful as a compounded failure test, but it cannot isolate network-size generalization. |
| C26 | The X36-hard seed-7 static prefix needs 1417.08 s to reach t=75 and is already unhealthy there: E-OSPA is 107.3636/150 and cardinality error is 12.3056/24. Across 33 valid actions, current task risk spans only 1.55% and its best surrogate gain over static is 0.736%. | High | E36 | Runtime is machine-specific. The normalized tracking/cardinality values show that topology selection is being tested after a much weaker state than M24, not that X36 is intrinsically impossible. |
| C27 | On X36-hard seed 7, all six arms are communication/edge/feasibility matched, but pure mean task risk improves E-OSPA by only 0.091% at one step and 0.358% over t=75–77; reliability reaches 0.109%, discrepancy degrades tracking, and mean-CVaR/CVaR do not beat mean. This fails the registered 5% practical-effect gate. | High | E37, E38 | The mean teacher is the numerical best observed arm in this bounded stress screen, but the effect is negligible and cannot support a scale-generalization claim. |
| C28 | X36-matched preserves 36 sensors, 24 targets, 44 edges and three replacements per step while matching the M24 sensing/load parameters more closely. On seeds 7/17/27 it passes the registered geometry gates with 0.36%–0.42% blackout, 31.1%–31.9% single-formation visibility, 67.7%–68.6% multi-formation visibility and 69–70 focus handovers. | High | E39, E40 | Geometry/visibility health is necessary but not sufficient; a filter-health snapshot and closed-loop screen remain required. |
| C29 | X36-matched improves the seed-7 t=75 static state over X36-hard but still fails the preregistered filter-health gate: its 1438.22 s prefix reaches E-OSPA 90.0227/150 and cardinality error 8.6944/24 (normalized 0.600/0.362). Its 33 valid actions have 2.574% task-risk spread and 1.263% surrogate gain over static. | High | E41 | The stronger action signal is not permission to run a paper-facing policy comparison from an unhealthy shared state. |
| C30 | X36-clean-scale keeps the X36 route/load/communication problem but scales M24-hard's per-sensor sensing envelope rather than matching aggregate visibility. On seeds 7/17/27 it has zero blackout, 9.3%–9.6% single-formation visibility, 90.5%–90.7% multi-formation visibility and 69–70 focus handovers. | High | E42, E43 | This is a clean scale diagnostic with greater natural sensing redundancy; C31/C32 separately test filter health and topology action value. |
| C31 | X36-clean-scale passes the seed-7 t=75 filter-health gate after a 1646.43 s prefix: E-OSPA is 46.8048/150 and cardinality error is 2.4444/24 (normalized 0.312/0.102). Its 33 valid actions have 11.237% task-risk spread and 1.595% surrogate gain over static. | High | E44 | This establishes a healthy, action-sensitive posterior checkpoint, not realized policy gain. |
| C32 | In the X36-clean-scale one-step screen, reliability, mean-CVaR and CVaR tie at E-OSPA 46.3778 versus 46.8048 static (0.912%); mean is 46.3788 and discrepancy is worse. Over t=75–77, mean is numerically best at 45.9258 versus 46.3716 (0.961%, 0.292% byte mismatch, zero infeasibility), but worsens the worst node from 63.8858 to 68.4314 and does not improve MAP-set disagreement. Reliability/mean-CVaR/CVaR are tail-safer but improve mean E-OSPA by only 0.241%. | High | E45, E46 | No tested X36-clean-scale arm satisfies both the 5% tracking-primary gate and tail safety. Mean is only the finite-pool numerical winner, not a validated scalable strategy. |
| C33 | Continuation caches now include the complete scenario and resolved fusion configurations, and reject same-name/same-time snapshots after either configuration drifts. | High | E47 | Existing local caches were migrated for this worktree; unversioned caches produced by older commits must be regenerated or migrated. |
| C34 | At the M24 seed-7 t=75 checkpoint, relaxing topology churn does not change the old teacher's approximately 2.35% surrogate gain, while repeating symmetric KLA for two and three rounds worsens task risk by 14.37% and 28.01% relative to one-round static fusion. | High | E48 | This diagnoses one checkpoint and the current KLA approximation; it does not prove that every multi-round consensus schedule is harmful. |
| C35 | A privileged receiver-specific directed teacher exposes a large action-space gap: realized one-step E-OSPA improves by 39.49% over static and 37.12% over local on M24, and by 36.38% over static and 33.01% over local on X36-clean-scale, while using fewer attempted payload bytes. | High | E48 | The teacher reads truth and is only an architecture diagnostic, not a deployable method or global upper bound. |
| C36 | The first deployable policy uses 21 truth-free posterior/geometry/link features. Receiver-held-out four-fold selection chooses k=40 and a 0.70 positive-neighbour confidence gate, with 15.21% mean selected gain and no negative selected receiver in this M24 checkpoint dataset. | High | E49, E50 | Receiver-held-out folds are not seed-, time-, or scenario-held-out; the model is fitted on M24 seed 7 at t=75. |
| C37 | On the M24 training checkpoint, the learned directed policy lowers E-OSPA from 25.5087 static and 24.5451 local to 15.4348, lowers MAP-set disagreement to 20.8647, and uses 41.27% of static attempted bytes with zero infeasibility. | High | E51 | This is an in-sample single-step result. Worst-node E-OSPA is 0.22% worse than static, although 0.40% better than local. |
| C38 | Without X36 labels or truth at inference, the same M24 model lowers X36-clean-scale E-OSPA from 46.8048 static and 44.4497 local to 41.6002, improves worst-node E-OSPA by 10.04% versus static without worsening local, and uses 20.83% of static attempted bytes with zero infeasibility. | High | E52 | This is a zero-shot cross-scale result but only one seed, one time step and a conditional shared checkpoint. |
| C39 | The directed online path preserves receiver×sender orientation, enforces physical and directed-message budgets, validates nonnegative row-normalized receiver-specific KLA weights, and its deployment feature extractor is invariant to changes in stored target truth. | High | E49, E53 | Connectivity is intentionally not required for each instantaneous directed routing graph; local fallback is an admissible action. |

## Evidence Ledger

| ID | Type | Source or artifact | What it supports | Strength |
|:--|:--|:--|:--|:--:|
| E1 | code | `RUN/GA/runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare.m:416-478` | C1: `numberOfSensors=8`, 8-node drop tiers, finite-FoV range 60000, and the fixed 4+4 builder call. | strong |
| E2 | code | `RUN/GA/runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare.m:1744-1795` | C1: explicit eight-sensor guard, two groups of four, fixed pairings, and fixed two-group initial states. | strong |
| E3 | experiment report | `docs/EFFECTIVE_KLA_GRAPH_VALIDATION_STATUS_CN.md:121-174` | C2/C7: N50 dynamic full used 6.1% more bytes and worsened consensus OSPA by 11.1%; dynamic light was dominated by static light. | strong |
| E4 | experiment report | `docs/DUAL_THRESHOLD_EVENT_TRIGGER_RESEARCH_CN.md:235-253` | C2/C7: a task-aware cross-layer score improved short-horizon consensus but worsened bytes and local tracking, leaving no validated Pareto gain. | medium |
| E5 | paper | DOI `10.1109/TCYB.2021.3087521`; author-uploaded full text at https://www.researchgate.net/publication/353116264_Consensus-Based_Labeled_Multi-Bernoulli_Filter_for_Multitarget_Tracking_in_Distributed_Sensor_Network | C3: the representative different-FoV consensus-LMB simulation reports sensors 1–4. | medium |
| E6 | paper | DOI `10.1109/TSP.2022.3154227`; author-uploaded full text at https://www.researchgate.net/publication/358614637_Consensus-Based_Labeled_Multi-Bernoulli_Filter_with_Event-triggered_Communication | C3: the representative event-triggered consensus-LMB simulation uses 9 sensors, up to 4 targets, and 50 Monte Carlo trials. | strong |
| E7 | paper | CVPR 2020 primary page: https://openaccess.thecvf.com/content_CVPR_2020/html/Liu_When2com_Multi-Agent_Perception_via_Communication_Graph_Grouping_CVPR_2020_paper.html | C3: learned communication-graph work identifies fully connected communication as quadratic in the number of agents. | medium |
| E8 | command | Octave pchip trajectory check recorded below | C4: continuous adjacent formation-center distances peak at 778.2 m; adding two 38.5 m jittered formation radii gives an 855.2 m conservative bound, below 900 m. Maximum sampled center speed and acceleration are also below the proposed limits. | strong |
| E9 | code | `multisensorLmb/runEventTriggeredDistributedLmbFilter.m:968-1007,1124-1141` | C5: no finite candidate edges trigger an all-to-all fallback; the distance-balanced helper requires exactly two equal groups with at most seven nodes each. | strong |
| E10 | code | `multisensorLmb/gaLmbTrackMerging.m:5-9,53-99,150-164` | C6: the code explicitly calls the merge crude, moment-matches each GM to one Gaussian, and writes back one component. | strong |
| E11 | design artifact | `docs/LEARNING_AUGMENTED_DYNAMIC_TOPOLOGY_SCENARIO_DESIGN_CN.md` | C7: separates D12 exact diagnosis, M24 main evaluation, X36 scale, strong analytic baselines, and stop/go gates. | medium |
| E12 | code | `common/buildDynamicTopologyScenarioConfig.m`, `common/generateDynamicTopologyScenarioInputs.m`, `common/buildDynamicTopologyGraphs.m`, `common/validateDynamicTopologyScenario.m` | C8: data-only presets, paired inputs, physical/static/candidate graphs and hard validation. | strong |
| E13 | test | `tests/test_dynamic_topology_scenarios.m`; command in Verification Record | C8–C10: all registered presets validate; D12 has 48 fourteen-edge candidates; scheduled births, time-varying loss, attempted bytes, fail-closed topology, KLA boundary cases and exact-callback smoke pass. | strong |
| E14 | code | `multisensorLmb/runEventTriggeredDistributedLmbFilter.m` | C9: S×S×T loss, external policy callback, non-physical-edge rejection, infeasibility diagnostics and attempted-payload accounting. | strong |
| E15 | code | `multisensorLmb/buildMixtureAwareKlaReferenceConfig.m`, `multisensorLmb/fuseLmbPosteriorsByLabel.m` | C10: the controlled reference enables multi-component fusion and mixture-aware existence normalization while documenting componentwise power approximation. | strong |
| E16 | experiment report | `RUN/GA/dynamic_topology/smoke_v2/DYNAMIC_TOPOLOGY_ORACLE_GAP_D12_HANDOVER_N1_20260725_131647.md` | C11: corrected 100 m cutoff, byte-matched 8-step arms, no early-window practical oracle gap, and runtime estimates. | weak |
| E17 | experiment report | `RUN/GA/dynamic_topology/full_n3/DYNAMIC_TOPOLOGY_FINDINGS_CN.md`, `RUN/GA/dynamic_topology/full_n3/DYNAMIC_TOPOLOGY_RECORDS.csv`, `RUN/GA/dynamic_topology/full_n3/DYNAMIC_TOPOLOGY_ORACLE_GAP_D12_HANDOVER_N3_20260725_153939.md` | C12/C13: paired focus-window results, machine-readable per-arm records, bytes, infeasibility, churn, candidate diversity and evidence boundaries for seeds 7/17/27. | strong |
| E18 | code | `common/buildDynamicTopologyGraphs.m:27-30,174-188` | C14: the fixed candidate minimizes mean edge distance plus 0.25 times worst edge distance; no filter-performance objective is evaluated. | strong |
| E19 | command | Independent CSV recomputation recorded in the Verification Record | C12/C13: per-seed focus improvements, byte mismatch and candidate diversity were recomputed from the tracked per-arm export. | strong |
| E20 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_SCENE_DIFFICULTY_20260725_173202.md` | C15: three-seed hard-scene blackout, visibility, handover, close-encounter, ownership and blockage metrics. | strong |
| E21 | test | `tests/test_dynamic_topology_scenarios.m`; command in Verification Record | C15–C17: hard-scene gates, invalid narrow-FoV rejection, scalable candidate and projected-policy constraints, task-risk ordering, switch-penalty invariance, fixed-reference stability and eligible-only scoring equivalence. | strong |
| E22 | code | `common/buildDynamicTopologyCandidatePool.m` | C16: exact D12 and projected M24/X36 candidate construction with safety constraints. | strong |
| E23 | code | `common/evaluateLmbTopologyTaskRisk.m`, `common/selectCounterfactualTopologyTeacher.m` | C17: task-only current/open-loop risk and separate deployment switch penalty. | strong |
| E24 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_TEACHER_SIGNAL_D12_HARD_N1_20260725_172736.md` | C18: t=30, horizon 0/3/6, nonzero action spread but identical current/predictive selection. | medium |
| E25 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_TEACHER_SIGNAL_D12_HARD_N1_20260725_173326.md` | C18: t=40/60/80, horizon 0/10/20, identical current/predictive selections and behavior-trajectory health metrics. | strong |
| E26 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_TEACHER_SIGNAL_D12_HARD_N1_20260725_174643.md` | C19: t=30 future-measurement H3 rollout, 2.07% spread, 1.50% gain over geometry-static, no action change, 93.32 s teacher time. | strong |
| E27 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_TEACHER_SIGNAL_D12_HARD_N1_20260725_175200.md` | C19: t=60 future-measurement H3 rollout with three-step candidate persistence, 4.27% spread, 2.71% gain over geometry-static, no action change. | strong |
| E28 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_ORACLE_GAP_D12_HARD_N1_20260725_181109.md` | C20: paired 60-step geometry-static, posterior-discrepancy and pure current task-risk results with tracking, consensus, bytes, churn and feasibility. | strong |
| E29 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_ORACLE_GAP_D12_HARD_N1_20260725_181426.md` | C21: 60-step registered fixed-candidate-16 tracking, consensus, byte and feasibility control. | strong |
| E30 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_TEACHER_SIGNAL_M24_HARD_N1_20260725_183112.md` | C22: cached 75-step M24 behavior cost, 28 valid candidate actions, current-task-risk action spread, static-relative surrogate gain and teacher scoring time. | strong |
| E31 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260725_215848.md` | C23: paired one-step static/mean/mean-CVaR/CVaR realized metrics and action agreement. | strong |
| E32 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260725_221559.md` | C24: paired six-step static, reliability, discrepancy, mean-task-risk and mean-CVaR screen. Its churn column excludes the prefix boundary and is superseded by E33 for the winner. | strong |
| E33 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260725_222820.md` | C24: boundary-corrected paired confirmation of static versus the winning mean-task-risk arm. | strong |
| E34 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260725_225246.md` | C24: boundary-aware mean-versus-pure-CVaR confirmation; pure CVaR matches mean-CVaR's realized metrics and remains weaker than pure mean risk. | strong |
| E35 | code | `common/buildDynamicTopologyScenarioConfig.m` | C25: exact M24-hard, X36-hard, X36-matched and X36-clean-scale sensing, load, scale, communication and gate parameters. | strong |
| E36 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_TEACHER_SIGNAL_X36_HARD_N1_20260725_234935.md` | C26: X36-hard cached-prefix runtime, failed normalized behavior-health gate, valid action count, surrogate spread and static-relative surrogate gain. | strong |
| E37 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_X36_HARD_T75_N1_20260725_230827.md` | C27: constraint-matched six-arm one-step realized comparison and practical-effect stop. | strong |
| E38 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_X36_HARD_T75_N1_20260725_233726.md` | C27: constraint-matched six-arm t=75–77 comparison and practical-effect stop. | strong |
| E39 | code/test | `common/buildDynamicTopologyScenarioConfig.m`, `tests/test_dynamic_topology_scenarios.m` | C28: separate X36-matched preset and preset/difficulty/candidate-pool regression coverage. | strong |
| E40 | command | Three-seed X36-matched geometry audit recorded in the Verification Record | C28: exact blackout, visibility, handover, close-encounter, ownership and blockage values. | strong |
| E41 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_TEACHER_SIGNAL_X36_MATCHED_N1_20260726_000437.md` | C29: X36-matched runtime, failed normalized behavior-health gate and current-task-risk action signal. | strong |
| E42 | code/test | `common/buildDynamicTopologyScenarioConfig.m`, `tests/test_dynamic_topology_scenarios.m` | C30: separate X36-clean-scale preset and preset/difficulty/candidate-pool regression coverage. | strong |
| E43 | command | Three-seed X36-clean-scale geometry audit recorded in the Verification Record | C30: exact blackout, visibility, handover, close-encounter, ownership and blockage values. | strong |
| E44 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_TEACHER_SIGNAL_X36_CLEAN_SCALE_N1_20260726_003744.md` | C31: X36-clean-scale runtime, passed normalized behavior-health gate and current-task-risk action signal. | strong |
| E45 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_X36_CLEAN_SCALE_T75_N1_20260726_005230.md` | C32: constraint-matched six-arm one-step comparison and practical-effect stop. | strong |
| E46 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_X36_CLEAN_SCALE_T75_N1_20260726_013455.md` | C32: constraint-matched six-arm t=75–77 comparison, numerical winner, tail failure and practical-effect stop. | strong |
| E47 | code/test | `RUN/GA/runDynamicTopologyTeacherSignalScreen.m`, `RUN/GA/runDynamicTopologyOracleGapScreen.m`, `tests/test_dynamic_topology_scenarios.m` | C33: stores/validates scenario and fusion snapshots and deterministically rejects caches after changing either measurement noise or mixture component count. | strong |
| E48 | diagnostic/code | `docs/DIRECTED_TASK_ROUTING_KEY_FINDING_CN.md`, `common/fuseLmbNetworkByTopologySchedule.m`, `common/selectDirectedTaskRoutingTeacher.m` | C34/C35: repeated-symmetric-fusion falsification and receiver-specific privileged action-space results at the shared M24/X36 checkpoints. | medium |
| E49 | code | `common/computeDirectedRoutingFeatures.m`, `common/fitDirectedRoutingKnnModel.m`, `common/selectFeatureDirectedRoutingPolicy.m`, `RUN/GA/trainDirectedRoutingPolicy.m` | C36/C39: truth-free deployable feature contract, receiver-held-out model selection, inference routing and reproducible artifact generation. | strong |
| E50 | model/data report | `RUN/GA/dynamic_topology/DIRECTED_ROUTING_MODEL_M24_T75.md`, `RUN/GA/dynamic_topology/cache/directed_teacher_m24_hard_seed7_t75.mat`, `RUN/GA/dynamic_topology/models/directed_routing_knn_m24_t75.mat` | C36: 552 edge examples, selected k/confidence gate and receiver-held-out metrics; the tracked model embeds its standardized training examples and labels. | strong |
| E51 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260726_031637.md` | C37: paired local/static/learned M24 one-step metrics, directed routes, attempted bytes, strict tail readout and infeasibility. | strong |
| E52 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_X36_CLEAN_SCALE_T75_N1_20260726_032238.md` | C38: zero-shot M24-model transfer to X36 with paired local/static metrics, tail safety, communication ratio and infeasibility. | strong |
| E53 | test | `tests/test_dynamic_topology_scenarios.m`; command in Verification Record | C39: exact sender→receiver accounting, custom fusion weights, directed budget feasibility, truth-invariant features and model-artifact checks. | strong |

Initial arithmetic check:

```bash
node - <<'NODE'
const W = {
 G1:[[-520,-180],[-340,-120],[-120,80],[260,200],[520,180]],
 G2:[[-180,520],[-120,340],[80,120],[200,-260],[180,-520]],
 G3:[[520,180],[340,120],[120,-80],[-260,-200],[-520,-180]],
 G4:[[180,-520],[120,-340],[-80,-120],[-200,260],[-180,520]],
};
const ring=[['G1','G2'],['G2','G3'],['G3','G4'],['G4','G1']];
let max=0;
for (const [a,b] of ring) {
  const ds=W[a].map((p,k)=>Math.hypot(p[0]-W[b][k][0],p[1]-W[b][k][1]));
  max=Math.max(max,...ds);
}
console.log(max, max+2*35*1.1);
NODE
```

Representative output:

```text
max center distance = 778.2 m
worst endpoint bound with 10% radius jitter = 855.2 m < 900 m
```

Continuous pchip check for E8:

```bash
octave --quiet <<'OCT'
tk = [1,40,80,120,160];
W = zeros(4,2,5);
W(1,:,:) = [-520,-340,-120,260,520; -180,-120,80,200,180];
W(2,:,:) = [-180,-120,80,200,180; 520,340,120,-260,-520];
W(3,:,:) = [520,340,120,-260,-520; 180,120,-80,-200,-180];
W(4,:,:) = [180,120,-80,-200,-180; -520,-340,-120,260,520];
t = 1:160; P = zeros(4,2,160);
for g=1:4
  P(g,1,:) = ppval(pchip(tk, squeeze(W(g,1,:))'), t);
  P(g,2,:) = ppval(pchip(tk, squeeze(W(g,2,:))'), t);
end
maxSpeed = 0; maxAccel = 0;
for g=1:4
  xy = squeeze(P(g,:,:)); vel = diff(xy,1,2); acc = diff(vel,1,2);
  maxSpeed = max(maxSpeed, max(sqrt(sum(vel.^2,1))));
  maxAccel = max(maxAccel, max(sqrt(sum(acc.^2,1))));
end
ring = [1,2;2,3;3,4;4,1]; maxCenter = 0;
for e=1:4
  dxy = squeeze(P(ring(e,1),:,:) - P(ring(e,2),:,:));
  maxCenter = max(maxCenter, max(sqrt(sum(dxy.^2,1))));
end
fprintf('%.4f %.4f %.4f %.4f\n', maxSpeed, maxAccel, maxCenter, maxCenter+2*35*1.1);
OCT
```

Representative output:

```text
11.1655 0.3845 778.2031 855.2031
```

## Verification Record

**Independence status: self-check only.** The implementation and results are
deterministically tested, but no independent agent verified this checkpoint.

Checks performed:

- inspected the exact hard-coded scenario, topology selection, diagnostics allocation, and GA merge call path;
- compared the proposed scale with full-text simulation details from two representative LMB papers;
- recomputed all complete-graph pair counts: N=8/12/24/36/48 gives 28/66/276/630/1128 pairs;
- recomputed the D12 enumerations: 48 formation-tree bridge topologies and 64 three-bridge-cycle topologies;
- checked the proposed M24 formation-ring waypoint distances and found that the initial 850 m radius was too tight once 10% formation-radius jitter was included; revised it to 900 m;
- evaluated the full 160-step pchip interpolation in Octave: maximum center speed 11.1655 m/s, acceleration 0.3845 m/s², adjacent-center distance 778.2031 m, and conservative sensor-pair bound 855.2031 m;
- ran `git diff --check`, which passed without whitespace errors.
- ran `test_dual_threshold_event_trigger`, which passed after the core changes;
- ran `test_dynamic_topology_scenarios`, which passed all preset, difficulty,
  candidate-pool, projected-policy, task-risk, label-invariance, safety,
  accounting, fusion-boundary and D12 callback checks;
- reproduced M24 limits from the implemented generator: 24 sensors, 30 static
  edges, maximum speed 11.45 m/s, acceleration 0.86 m/s² and target speed
  13.80 m/s;
- reproduced X36 limits: 36 sensors, 45 static edges, maximum speed
  6.41 m/s, acceleration 0.41 m/s² and target speed 13.67 m/s;
- ran an 8-step six-arm smoke with attempted-byte mismatch below 1% for the
  consensus oracle comparison after adding a per-step byte feasibility gate.
- ran exactly three paired D12 trials, seeds `[7,17,27]`, through step 95;
  the tracked aggregate report and paired findings are E17;
- independently reloaded the saved MAT summary and recomputed each seed's
  focus tracking improvement, posterior-disagreement improvement,
  attempted-byte mismatch, infeasibility rate and distinct-candidate count;
- reran `test_dynamic_topology_screen_analysis` after the runner changes;
- ran a D12-hard seed-7 60-step paired closed-loop screen for geometry-static,
  posterior-discrepancy and pure current task risk, then checked registered
  fixed candidate 16 as a stronger in-sample static control;
- ran the M24-hard seed-7 behavior trajectory through t=75 and reused its
  cached posterior to verify 28 selection-valid actions, 15.373% current-risk
  spread and 24.70 s teacher-scoring time.
- fixed the sensor-risk aggregation path so mean-CVaR/CVaR operate on the
  exposed per-node risks rather than reusing a scalar mean, then manually
  recomputed the mean-CVaR objective in the deterministic regression test;
- verified that a run resumed from a cached post-update local posterior
  reproduces the corresponding full-run labels, means, covariances, active
  topology and attempted bytes exactly from the continuation time onward;
- ran the M24-hard seed-7 t=75 one-step aggregation screen, the t=75–80
  five-arm screen, and a boundary-aware two-arm confirmation of the winning
  mean-task-risk strategy;
- added the previously missing pure-CVaR six-step arm; its realized metrics
  match mean-CVaR and remain weaker than pure mean risk;
- generated the 1417.08 s X36-hard static prefix and ran one- and three-step
  six-arm conditional screens; pure mean risk is numerically best but remains
  below 0.4% E-OSPA gain;
- compared M24-hard and X36-hard configuration fields and found that the
  original X36 stress preset changes both scale and sensing difficulty;
- introduced X36-matched as a separate scale-only preset and checked its
  geometry gates on seeds 7/17/27;
- ran the X36-matched t=75 behavior/action screen; it improves over X36-hard
  but still fails both normalized filter-health limits;
- added X36-clean-scale to isolate network/target scale without reducing the
  normalized per-sensor sensing envelope, and checked its geometry gates on
  seeds 7/17/27;
- generated the X36-clean-scale t=75 cache, verified that it passes the
  normalized filter-health gate, and ran one- and three-step six-arm screens;
- independently recomputed the three-step mean-teacher gain (0.961392%),
  reliability gain (0.240969%), mean-teacher worst-node degradation
  (7.115215%) and attempted-byte mismatch (0.291976%) from the report values;
- added scenario and fusion configuration snapshots to continuation caches,
  deterministic stale-cache rejection tests for both drift types, and
  migrated the four ignored local checkpoint caches;
- ran a one-step constraint-selection smoke where a local-only arm had lower
  E-OSPA but zero attempted bytes; the constraint-aware decision correctly
  excluded it and retained the communication-matched static arm.
- verified at M24 t=75 that removing the churn restriction leaves the old
  teacher action/gain unchanged, while two and three synchronous symmetric
  KLA rounds worsen the task-risk diagnostic;
- integrated directed receiver×sender topology selection and receiver-specific
  fusion weights into the online filter, including physical-edge, message
  budget, inactive-weight and nonnegative row-normalization checks;
- generated the M24 directed-teacher dataset, refit the tracked kNN artifact,
  and reproduced the receiver-held-out choice `k=40`, gain threshold `0`,
  positive-neighbour confidence `0.70`, mean selected gain `15.2095%`, and
  zero negative selected receivers;
- extended `test_dynamic_topology_scenarios` with exact one-way sender→receiver
  accounting, custom KLA-weight, truth-invariant feature and model-artifact
  checks; the complete test passed;
- reran paired local/static/learned one-step continuations from the common
  M24 and X36 t=75 caches. The final reports are E51/E52 and independently
  expose the directed Pareto readout instead of excluding sparse arms through
  the old equal-edge gate.

Falsification findings incorporated into the draft:

- scaling the old driver by editing `numberOfSensors` is invalid;
- equal edge counts do not imply exactly equal attempted bytes when sender posterior sizes vary, so the design now requires matched-byte curves;
- an exact future/global oracle must not be presented as deployable;
- a fixed static baseline must remain physically feasible throughout the scene, otherwise the comparison would be biased;
- current projected-Gaussian fusion is explicitly excluded from paper-level mixture-aware claims.
- the inherited `eC=5 m` makes large-scene E-OSPA saturate and was replaced by
  preset-scale cutoffs (D12 100 m, M24 120 m, X36 150 m);
- an exact enumeration of one-step actions is not an exact full-horizon
  oracle, because its actions change subsequent posteriors.
- the three-seed screen falsified the proposed one-step teacher: both
  diagnostic arms were worse than posterior-discrepancy on both focus metrics
  for all seeds;
- the old three-seed fixed comparison arm is geometry-selected rather than
  an exhaustive offline tracking optimum; candidate 16 strengthens the seed-7
  screen but is still an in-sample shortlist rather than a train-selected,
  held-out strong-static baseline;
- a dominated diagnostic reference cannot produce a meaningful
  static-to-oracle gain-capture ratio; the analyzer now reports this as a
  teacher/reference failure instead of analytic sufficiency.
- more symmetric KLA rounds are not a substitute for better routing at the
  audited M24 checkpoint; repeated full-graph fusion progressively dilutes
  useful local posteriors;
- an equal-undirected-edge gate is inappropriate for a sparse directed Pareto
  arm. The revised report keeps that old gate for like-for-like undirected
  comparisons and separately requires directed routing to beat both static
  and local without exceeding static attempted bytes.

Unverified:

- D12 current-task-teacher direction consistency beyond seed 7;
- the best train-selected fixed D12 graph and its held-out performance;
- whether the directed learned policy remains beneficial over multiple
  feedback steps after t=75;
- M24 seed/time-held-out direction consistency, full-episode behavior, and
  the complete M24/X36 runtime-memory curve;
- X36-clean-scale direction consistency beyond seed 7 and any full-episode,
  multi-seed, or X36-label-trained comparison;
- whether a GNN improves held-out routing over the inspectable kNN baseline;
- whether the proposed effect-size gates are appropriately calibrated.

## Risk and Escalation

If the scenario is biased or the single-checkpoint effect does not survive
closed-loop feedback, subsequent GNN results could be publishable-looking but
scientifically uninformative, and long Monte Carlo runs could be wasted.
Before a paper claim or a long sweep, author review should confirm the scale
ladder, communication radius, target count, directed-message budget, tail
safety rule and Gate B/C thresholds. A domain review is also required when the
mixture-aware LMB-KLA reference is specified.

This package is self-check only and does not authorize a paper-facing claim
without that review.

## Reproducibility

Repository, audited source baseline, and scenario checkpoint:

```text
/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/learned-dynamic-topology-scenarios
audited source baseline: 282ca8180510315424dbb488ce7cfd80e624115f
scenario checkpoint: 2c75193
three-trial implementation checkpoint: b63fd2b
M24 continuation-strategy checkpoint: e052774
X36 screening-gate checkpoint: 570a6c5
```

Core inspection commands:

```bash
nl -ba RUN/GA/runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare.m | sed -n '416,478p;1744,1795p'
nl -ba multisensorLmb/runEventTriggeredDistributedLmbFilter.m | sed -n '968,1007p;1124,1141p'
nl -ba multisensorLmb/gaLmbTrackMerging.m | sed -n '1,100p;150,164p'
nl -ba docs/EFFECTIVE_KLA_GRAPH_VALIDATION_STATUS_CN.md | sed -n '121,174p'
nl -ba docs/DUAL_THRESHOLD_EVENT_TRIGGER_RESEARCH_CN.md | sed -n '235,253p'
octave --quiet --eval "setPath; addpath('tests'); addpath(fullfile('RUN','GA')); test_dynamic_topology_screen_analysis"
octave --no-gui --quiet --eval "addpath(genpath(pwd)); test_dynamic_topology_scenarios; test_dynamic_topology_screen_analysis; test_dual_threshold_event_trigger;"
git diff --check
```

Directed-routing model and final one-step checkpoint screens for E49–E52:

```bash
octave --quiet --eval "
addpath(genpath(pwd));
trainDirectedRoutingPolicy(struct('regenerateDataset',false));"

octave --quiet --eval "
addpath(genpath(pwd));
o=struct( ...
  'maxTimeSteps',75, ...
  'continuationStartTime',75, ...
  'armNames',{{'local','robust-static', ...
    'learned-directed-routing'}});
runDynamicTopologyOracleGapScreen('m24-hard',7,o);
runDynamicTopologyOracleGapScreen('x36-clean-scale',7,o);"
```

Representative output:

```text
test_dynamic_topology_scenarios passed
M24: status=directed-routing-screening-gain-tail-caveat
      E=15.434821 gain_static=39.492% gain_local=37.117%
      bytes_ratio=41.266% worst_static=-0.220% routes=24
X36: status=directed-routing-screening-gain
      E=41.600155 gain_static=11.120% gain_local=6.411%
      bytes_ratio=20.834% worst_static=10.037% routes=18
```

X36-clean-scale behavior checkpoint and six-arm conditional screen for
E44–E46:

```bash
octave --no-gui --quiet --eval "
addpath(genpath(pwd));
cacheDir=fullfile('RUN','GA','dynamic_topology','cache');
teacherOpts=struct( ...
  'snapshotTimes',75, ...
  'teacherHorizonSteps',0, ...
  'behaviorCacheDirectory',cacheDir);
runDynamicTopologyTeacherSignalScreen( ...
  'x36-clean-scale',7,teacherOpts);"

octave --no-gui --quiet --eval "
addpath(genpath(pwd));
cacheDir=fullfile('RUN','GA','dynamic_topology','cache');
arms={'robust-static','reliability','discrepancy', ...
  'teacher-current','teacher-current-balanced', ...
  'teacher-current-cvar'};
screenOpts=struct( ...
  'maxTimeSteps',77, ...
  'continuationStartTime',75, ...
  'armNames',{arms}, ...
  'behaviorCacheDirectory',cacheDir);
runDynamicTopologyOracleGapScreen( ...
  'x36-clean-scale',7,screenOpts);"
```

Representative terminal summaries:

```text
health=1 normE=0.312032 normCard=0.101852
gain=1.594611 spread=11.237081 candidates=33
status=stop-negligible-observed-gain eligible=6
best=Pure current task-risk teacher gain=0.961392
bytesMismatch=0.291976
```

X36-matched three-seed geometry audit for E40:

```bash
octave --no-gui --quiet --eval "
addpath(genpath(pwd));
for seed=[7,17,27]
  rng(seed);
  c=buildDynamicTopologyScenarioConfig('x36-matched');
  [s,~]=generateMultiFormationTrajectories(c);
  [t,~]=generateCorridorTargetTrajectories(c);
  g=buildDynamicTopologyGraphs(c,s);
  v=validateDynamicTopologyScenario(c,s,t,g);
  d=v.difficulty;
  fprintf('%d %.4f %.4f %.4f %d %.4f %.4f %.4f\n', ...
    seed,d.blackoutFraction,d.singleFormationFraction, ...
    d.multiFormationFraction,d.focusHandovers, ...
    d.focusCloseEncounterTimeFraction, ...
    d.formationOwnershipEntropy, ...
    d.blockageFocusOverlapFraction);
end"
```

Representative output:

```text
7  0.0042 0.3187 0.6771 69 0.8372 0.9995 0.7326
17 0.0036 0.3107 0.6857 69 0.8372 0.9996 0.7326
27 0.0039 0.3128 0.6833 70 0.8372 0.9996 0.7326
```

X36-clean-scale uses the same command with preset
`x36-clean-scale`. Representative E43 output:

```text
7  0.0000 0.0955 0.9045 69 0.8372 0.9995 0.7326
17 0.0000 0.0938 0.9062 69 0.8372 0.9996 0.7326
27 0.0000 0.0929 0.9071 70 0.8372 0.9996 0.7326
```

Independent E19 recomputation from the tracked CSV:

```bash
python3 - <<'PY'
import csv
from collections import defaultdict

path = "RUN/GA/dynamic_topology/full_n3/DYNAMIC_TOPOLOGY_RECORDS.csv"
rows = defaultdict(dict)
with open(path, newline="") as handle:
    for row in csv.DictReader(handle):
        rows[int(row["seed"])][row["arm_mode"]] = row
for seed in sorted(rows):
    arm = rows[seed]
    static, disc = arm["robust-static"], arm["discrepancy"]
    consensus, truth = arm["oracle-consensus"], arm["oracle-truth"]
    improve = lambda a, b, key: 100 * (float(a[key]) - float(b[key])) / float(a[key])
    print(
        seed,
        f"{improve(static, disc, 'focus_eospa'):.8f}",
        f"{improve(static, disc, 'focus_posterior_disagreement'):.8f}",
        f"{100 * abs(float(disc['attempted_bytes']) - float(static['attempted_bytes'])) / float(static['attempted_bytes']):.8f}",
        f"{improve(disc, consensus, 'focus_eospa'):.8f}",
        f"{improve(disc, truth, 'focus_eospa'):.8f}",
        disc["distinct_candidates"],
        consensus["distinct_candidates"],
        truth["distinct_candidates"],
    )
PY
```

Representative output:

```text
7  9.16157262  8.82349965  1.37262056 -11.22794067 -2.77653676 34 6 1
17 6.39984278  8.91084635  0.61052319  -7.77930568 -7.03489390 35 5 5
27 4.74491841 14.54526797  0.19666252  -4.37257133 -4.37257133 28 2 2
```

Evidence-package lint:

```bash
python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py docs/DYNAMIC_TOPOLOGY_SCENARIO_EVIDENCE.md
```

## Open Issues

1. Generate current-task-risk labels on D12 training seeds and test whether
   locally available graph/posterior features can predict the privileged
   ranking without truth at deployment.
2. Evaluate all 48 fixed D12 candidates on training seeds, freeze the best
   robust fixed graph, and compare it on held-out paired seeds.
3. The componentwise powered-GM reference needs a stronger numerical
   comparison before it can support a paper-level density claim.
4. The implemented generator must repeat route, speed, acceleration and
   static-backbone checks after any trajectory change.
5. Use cached sparse posterior snapshots for M24/X36 label generation; the
   M24 75-step behavior trajectory already costs 559.37 s on the current
   machine.
6. Repeat the conditional M24 comparison on additional paired seeds and a
   longer window before treating the 8.21% screen as an effect estimate.
7. Retain X36-hard/X36-matched as stress negatives. X36-clean-scale passes
   filter health but fails both the 5% practical-effect and tail-safety gates;
   do not extend this arm set to longer or multi-seed runs without a method
   change.

## Recommendation

Retire the old consensus/truth-composite teacher and the redundant open-loop
look-ahead labels. Keep pure current task risk as the privileged supervision
candidate: it has a positive D12 closed-loop screening result against both
posterior-discrepancy and an in-sample stronger fixed graph (C20/C21), and it
retains substantial action separation and the best observed conditional
closed-loop result at M24 scale (C22–C24). Mean-CVaR and CVaR add no observed
benefit at the registered M24 checkpoint.

The next research stage remains label-pipeline and imitation validation, not a
paper-level GNN claim. X36-hard must not be used as the clean scale claim:
its best observed gain is negligible and its sensing model is simultaneously
harder (C25–C27). X36-matched also fails the filter-health gate despite a
larger action signal (C28/C29). X36-clean-scale resolves health and action
separation, but the full six-arm screen still has no practically useful,
tail-safe winner (C30–C32). Retain pure mean task risk as the M24 privileged
teacher only; do not claim X36 strategy generalization from the current
single-round architecture.
In parallel, select the
strongest fixed graph only on training seeds, train a policy using locally
available inputs to imitate current-task-risk rankings, freeze it, and then
run paired held-out tests.
