# Evidence package: multi-formation dynamic-topology scenario

## Question

After replacing the legacy 4+4 setup with the D12/M24/X36 ladder, does
receiver-specific dynamic KLA scheduling retain deployable value on M24 and
X36, and which analytic or learned design is justified by the current
evidence?

The supported decision is to keep the scenario ladder, retire the first
single-snapshot kNN transfer story, and reclassify receiver-complete
reliability routing at source weight 0.50 as a strong fixed-index directed
control rather than a dynamic method. Learning remains unvalidated, and any
new posterior-aware policy must exceed fixed-index, fixed-cycle and
round-robin controls, including a phase-registered all-physical schedule when
cross-formation routes are used, under the registered gain, tail, dynamicity,
feasibility and communication gates.

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
  a common constraint-checked candidate-pool interface, and counterfactual task-risk
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
- paired three-step M24/X36 conditional continuations that expose posterior
  feedback, tail behavior, directed routes and attempted-byte ratios.
- a seed-17 kNN held-out continuation, reproducible feature-support audit,
  corrected Bernoulli-delivery joint-action teacher, M24/X36 reliability
  weight screens and compatibility falsification;
- scale-normalized joint sender-weight residual features, seeds-7/17 training,
  seed-11 selected-action calibration, explicit OOD fallback and a paired M24
  confirmation that the retained learner exactly falls back to its backbone.
- an all-time M24/X36 route-map audit, exact fixed-index clone, balanced fixed
  and round-robin controls at both backbone- and candidate-matched weights,
  all-physical phase-registered schedules, focus-aligned dynamicity
  diagnostics, and a first posterior-aware reliability-plus-source-quality
  development arm.

Excluded:

- an exhaustive performance sweep over all 48 fixed D12 graphs;
- a multi-seed held-out validation of the current-task-risk teacher;
- multi-seed, full-episode M24/X36 tracking validation and a complete
  runtime-memory scaling curve;
- multi-seed or full-episode X36-clean-scale validation;
- an exact arbitrary-GM density-power implementation;
- fresh scenario-held-out evaluation of any dynamic candidate; seed 27 has
  already been used for geometry auditing and is only a method-parameter
  canary;
- longer-window or full-episode validation of directed routing;
- a comprehensive systematic review of every distributed LMB paper.

## Risk Tier

**L3.** The old undirected one-step teacher has a confirmed stop finding, and
repeating symmetric KLA makes the M24 checkpoint worse. A receiver-specific
directed-routing action space still exposes useful signal, but held-out seed 17
falsifies the first kNN transfer story: it remains strong on M24 and improves
X36 over static, yet improves X36 over local by only 1.05%, below the registered
5% gate. Feature-distance auditing shows complete X36 support violation and
partial receiver coverage.

A simpler receiver-complete reliability policy with equal KLA weight gives
the clearest fixed directed-control signal. Over the conditional t=75–77 window,
it improves M24 by 19.51% over local and 22.56% over static on seeds 7/17
averaged; on X36 seeds 7/17 it improves local by 12.12%/9.84% and static by
18.24%/18.95%. The all-time route audit now shows that this arm always uses
the same per-formation sender map `[2,1,1,1,1,1]`, with zero within-window
route changes and no cross-formation messages. It is exactly the registered
fixed-index control, so the gains cannot be attributed to dynamic topology.
These are conditional development runs, payload-only accounting, and not a
full-episode or held-out effect estimate. The corrected
support-gated residual learner currently falls back on every calibration
receiver, so it has no demonstrated incremental value over the analytic
code path or the fixed-index control.

The first posterior-aware source-quality candidate also fails the corrected
D12 attribution screen. On seed 7 over t=1–3, enumerating all 11 phases of the
all-physical round-robin schedule at source weights 0.50 and 0.70 finds a
28.1670 E-OSPA control, versus 28.6241 for the candidate. The candidate is
1.6229% worse, is not tail-safe, and has no tracking-byte Pareto advantage.
This is a development falsification of the current scoring formula, not a
cross-scenario conclusion.

The current evidence authorizes strong-control registration and bounded
development of a genuinely posterior-aware route-changing policy. It does not
authorize a paper claim of dynamic-topology gain, learned scalable
superiority, a consensus guarantee, or total-communication savings.

## Claims

| ID | Claim | Confidence | Evidence IDs | Caveats |
|:--|:--|:--:|:--|:--|
| C1 | The current 4+4 driver is hard-coded for eight sensors and cannot represent the proposed multi-formation scale by changing one parameter. | High | E1, E2 | Core filter functions are more generic than the experiment driver. |
| C2 | The old 4+4 experiment does not provide positive evidence for dynamic topology; its final N50 dynamic arms were dominated or degraded. | High | E3, E4 | This is evidence about the old scene and current heuristic, not a proof that all dynamic topology is ineffective. |
| C3 | A 12/24/36 sensor ladder creates a substantially larger topology decision space while remaining a more defensible first step than jumping to 48+ sensors. | Medium | E5, E6, E7 | One M24 behavior runtime is now measured in C22; the full runtime-memory scaling curve remains unmeasured. |
| C4 | The proposed M24 formation-level ring can have a physically feasible static backbone with a 900 m communication radius along the specified pchip center trajectories. | High | E8 | The final implemented generator must reproduce this check after any parameter change. |
| C5 | The audited baseline topology fallback could violate the intended physical graph, and its bridge helper was specialized to two equal groups. | High | E9 | The new fail-closed handling and scalable constraint-checked candidate projection supersede this baseline behavior in C9/C16. |
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
| C40 | Over the M24 seed-7 t=75–77 conditional continuation, learned directed routing improves mean E-OSPA by 39.42% over static and 34.48% over local, improves worst-node E-OSPA by 18.61% and 17.50%, uses 30.93% of static attempted bytes, and has zero topology infeasibility. | High | E54 | The three-step window exposes posterior feedback but remains adjacent to the M24 training checkpoint and is not seed-held-out. |
| C41 | Over the X36-clean-scale seed-7 t=75–77 zero-shot continuation, the same M24 model improves mean E-OSPA by 16.91% over static and 10.69% over local, improves worst-node E-OSPA by 7.64% versus static without worsening local, uses 20.27% of static attempted bytes, and has zero topology infeasibility. | High | E55 | This is cross-scale and multi-step, but still a single random seed and conditional rather than full-episode evidence. |
| C42 | On held-out seed 17 over t=75–77, the first M24-trained kNN policy improves M24 E-OSPA by 22.62% over local, but improves X36 by only 1.05% over local despite an 11.05% gain over static. It therefore fails the registered X36 mean-tracking gate. | High | E56 | This is a conditional three-step continuation and tests one held-out seed, not a full-episode effect estimate. |
| C43 | The kNN transfer failure is accompanied by severe support shift: median nearest-neighbour distance rises from 1.165 in training to 15.067/15.489 on X36 seeds 7/17, all X36 queries cross at least one training feature range, and active receiver coverage falls to 18/36 and 16/36. | High | E57 | The deterministic audit uses stored t=75 posterior checkpoints; it diagnoses support and coverage rather than closed-loop tracking quality. |
| C44 | Receiver-complete reliability routing with KLA source weight 0.50 gives a cross-scale fixed directed-control signal: M24 seeds 7/17 average 18.6913 E-OSPA, 19.51% below local and 22.56% below static; X36 seeds 7/17 improve local by 12.12%/9.84% and static by 18.24%/18.95%. | High | E54, E55, E56, E58, E59, E64 | All values use the shared-static-prefix t=75–77 continuation. E64 proves the selected sender map is fixed, so this is not dynamic-topology evidence. Weight 0.50 is a registered control point, not a proven optimum. |
| C45 | Moment-compatibility routing at weight 0.50 improves the two-seed M24 mean from 18.6913 to 17.9412, but worsens the aggregate worst node from 33.0095 to 36.5745 and costs 17.14 policy seconds; on X36 seed 7 it worsens E-OSPA from 37.9153 to 40.4920. | High | E60 | The X36 arm was stopped after seed 7 because it was both worse and much slower; this rejects it as the default backbone but not every compatibility feature. |
| C46 | The directed teacher now models delivery as a Bernoulli event: successful delivery applies the registered KLA weight and failure retains the local posterior. It saves full-shape receiver-by-sender-by-weight conditional-risk, expected-risk and expected-gain tensors, with physical candidate entries populated. | High | E61 | The corrected exact expectation currently supports one message per receiver; multi-message sequential delivery remains unimplemented. |
| C47 | A 32-feature scale-normalized residual model trained on M24 seeds 7/17 and calibrated on seed 11 keeps 82.25% of calibration actions inside its support box. The one calibration block gives an empirical selected-action worst-overestimate correction of 1.4011 and permits zero backbone overrides. | High | E62 | This is a negative development result: one block cannot supply a 95% population guarantee, and no incremental learned benefit is demonstrated. |
| C48 | Over M24 seeds 7/17 at t=75–77, the retained support-gated residual policy exactly reproduces the reliability-w0.50 path—and therefore the fixed-index control—on E-OSPA, tail, bytes and routes. In the frozen report its aggregate policy time is 17.11 s versus 0.03 s for the control path, a 17.074 s increment. | High | E63, E64 | Exact fallback prevents an unsupported learned action, but it means the learner adds cost with no measured tracking, communication or dynamic-routing value. Runtime is machine-load dependent. |
| C49 | On M24-hard and X36-clean-scale, seeds 7/17/27 and all 160 steps, reliability-w0.50 has one unique route map, zero within-window receiver changes, zero cross-formation messages and the per-formation sender map `[2,1,1,1,1,1]`. Its routing and fusion weights exactly match `directed-fixed-index-w50`. | High | E64 | This is a route-attribution audit, not a new tracking experiment. It reclassifies earlier numerical gains as fixed directed-control gains. |
| C50 | On D12-handover seed 7 over t=1–3, the posterior-aware reliability-plus-source-quality candidate reaches 28.6241 E-OSPA, but a complete 11-phase all-physical round-robin sweep at weights 0.50/0.70 reaches 28.1670. The candidate is 1.6229% worse, is not tail-safe, and fails the registered attribution gate. | High | E65 | This is a single-seed, three-step development falsification of the current source-quality score. It does not establish M24/X36 behavior or identify the strongest possible frozen cross-formation map. |
| C51 | A receiver-complete formation-gateway proxy that requires both positive pairwise utility margin and absolute existence novelty improves corrected one-step teacher risk over phase-1 intra-formation round-robin by 6.01%/5.22%/7.24% on M24 t=75 seeds 7/11/17, with zero harmful selected gateway, two or three cross-formation overrides, and attempted-payload ratios of 0.9917/0.9936/1.0069. However, all three induced formation graphs are not weakly connected, so the corrected training, validation and all-observed proxy gates all fail. | High | E66 | All three snapshots are development evidence. Receiver coverage is not a connectivity guarantee; this is privileged one-step proxy risk, not closed-loop E-OSPA, consensus, X36 transfer or held-out validation. |
| C52 | In the phase-complete 15-arm M24 seed-7 t=75–77 closed loop, novelty-gated gateway v1 improves E-OSPA by only 1.6781% over the strongest matched control (phase-1 round-robin), worsens the worst node from 34.8109 to 35.1839, and fails the registered attribution gate. It uses 1.78% fewer attempted payload bytes, retains full receiver coverage and zero infeasibility, and changes its gateway-only map. Its formation graph is disconnected at every individual step but weakly connected over the three-step union. | High | E67 | This is a one-seed, three-step conditional falsification of v1's practical/tail claim after fixed/rotating phases 1–6 and the corrected link-aware control. It blocks M24 multi-seed and X36 expansion of this version. |
| C53 | A fixed-index-backbone strong formation cycle at w=0.70 is tail-safe on M24 seed 7 and improves E-OSPA from 19.2470 to 18.5164 at its best bridge weight 0.10, but the 3.80% gain is below the 5% gate. Larger bridge weights monotonically lose the benefit and eventually worsen tail performance. | High | E68 | This is a one-seed conditional development screen. It rejects per-step strong connectivity as the final action space; it is not a learned-policy or held-out result. |
| C54 | Joint-tree v4 replaces the per-step strong cycle with a rooted tree whose union with the previous formation graph must be strongly connected. The exact projection jointly optimizes sensor endpoints under a one-cross-send-per-sensor constraint. On M24 seed 7 the analytic scorer is tail-safe and reduces attempted bytes by 7.27%, but improves E-OSPA by only 0.07% over fixed-index w=0.70. | High | E69 | The graph construction executes correctly on this seed, but E73 shows that its every-step rooted-tree feasible set does not generally contain a 5% action. |
| C55 | At t=75, the privileged optimal tree relative to fixed-index w=0.70 has positive selected residuals and one-step gains of 7.27%/15.90%/9.93% on M24 seeds 7/11/17. In the seed-7 t=75–77 closed loop, privileged joint-tree v4 improves E-OSPA from 19.2470 to 17.9780 (6.59%), keeps the worst node at 34.6420, uses 1.66% fewer attempted bytes and has a strongly connected three-step formation union. | High | E70 | This is a limited truth-assisted architecture signal, not deployable evidence; E73 rejects using it to justify learning the unchanged every-step tree feasible set. |
| C56 | The earlier privileged-v4 teacher series cannot support its reported ridge/kNN/structured-ranking conclusions. At \(t>t_0\), the generator passed the sender-row/receiver-column diagnostic edge mask directly into a receiver-row/sender-column policy context, reversing previous-edge features and the previous/current connectivity constraint. The corrected generator shares the live-filter converter, writes isolated `ctxv2` files, and fail-closes on the new series contract. | High | E71 | The old `directed_teacher_oracle_v4_*` caches and their proxy reports are superseded, not negative model evidence. Direct action features remain truth-free, but privileged behavior makes the sampled posterior-state distribution truth-assisted; corrected-data conclusions are reported separately in E73. |
| C57 | Connected-tree v2 on X36 seed 7 improves round-robin w=0.40 E-OSPA by 3.6629%, improves the worst node from 61.2646 to 56.3264 and uses 0.9873% fewer attempted bytes, while preserving instantaneous weak connectivity. It does not reach the 5% gate and was not compared against the corrected fixed-index strong control in the same report. | High | E72 | This is a scale screen, not evidence that dynamic routing beats the strongest registered X36 baseline. |
| C58 | On the corrected ctxv2 M24 blocks (seeds 19/23/29/31/37, t=75:80), the exact joint-tree v4 projector falls below the 5% one-step proxy gate on 9/30 blocks and reaches -1.5963% at seed 23/t=77. Removing the previous/current union constraint leaves the same 9/30 failures and a -1.2429% minimum, so the every-step rooted-tree feasible set—not the learned scorer—is the primary ceiling. | High | E73 | These are truth-assisted one-step-risk labels on an oracle-v4 state distribution, not truth-free closed-loop tracking evidence. The preregistered audit fail-closes and writes no artifact. |
| C59 | A diagnostic byte-fair optional cross-edge matching oracle with rolling \(B=3\) strong connectivity gives six-step weighted proxy gains of 11.92%/5.43%/15.55%/30.22%/6.04% on corrected ctxv2 seeds 19/23/29/31/37; \(B=2\) remains below 5% on seeds 23 and 37. | Medium | E74 | The ILP is an action-space design calculation, not yet a committed online projector or truth-free closed-loop method. All five inspected seeds become development data; no learned artifact or X36 claim follows. |
| C60 | On the M24-hard seed-7 t=75–77 continuation, all six centralized joint-baseline counterfactual actions reduce posterior disagreement by 0.39%–10.49% at no more than 1.26% attempted-byte mismatch, yet every action worsens E-OSPA by 2.69%–25.01%. The least harmful mean-\(k=1\) action also slightly worsens the worst node, while larger or top-fraction actions degrade the worst node by as much as 30.36%. One-step posterior consensus is therefore not a sufficient tracking surrogate for this checkpoint, and these labels are rejected for deployable learning. | High | E75 | This is one design-seen seed and a three-step conditional continuation. The teacher is truth-free but centralized and offline because it reads all full network posteriors; its counterfactual routes also invoke the safety repair on one of three focus steps. The result falsifies this surrogate at the audited checkpoint, not every posterior-based multi-step objective. |
| C61 | The clean-commit v3 M24 state artifact contains 18 seed-time blocks and supports an 880-dimensional complete state-action representation with cross-channel co-occurrence and rolling-\(B=3\) topology summaries. Three relabellings per block change the representation by at most \(2.44\times10^{-15}\); all 16 distinct registered action/reference pairs are separated by at least 6.7509, the seed-7 observed-range violation is 2.56% against a 5% gate, the maximum standardized magnitude is 6.8571 against an 8.0 gate, and S8/G4 versus S12/G6 keeps the same dimension. The formal preflight therefore permits M24 multi-action H=3 data generation. | High | E76 | This verifies the reducer and frozen registered-action set, not broad proposal-bank collision versus return gaps, candidate coverage, critic accuracy, end-to-end extractor/projector equivariance or X36 support. Offline labels use privileged future outcomes. The truth-free critic assumes a centralized coordinator with all-node posterior summaries, geometry, link state, payload metadata and selected history; control-metadata traffic is not yet counted, so no end-to-end communication-saving claim follows. |
| C62 | On all 18 frozen M24 development states, the 28-direction truth-free bank produces 341 distinct complete rolling-\(B=3\) actions, of which 338 are repair-free and critic-selectable. Each state retains 12–23 distinct and 11–23 selectable actions under the 32-action cap. Three repaired action-24 graphs are retained only as matched references. Recomputing all 341 joint representations gives zero replay error; the minimum distance between different within-state graphs is 0.02089 and no pair collides at the \(10^{-6}\) threshold. The structural preflight therefore permits paired H=3 return generation. | High | E77 | This establishes construction, exact safety projection, graph deduplication and representation separation only. It does not establish top-K capture, candidate-oracle tracking gain, byte/tail admissibility, critic accuracy, X36 transfer or end-to-end communication savings. |
| C63 | Across all 18 frozen M24 development states, 338 selectable truth-free state-actions were replayed with state-matched common-random-number H=3 continuations. Under the frozen mean/tail/2%-byte/B3/emergency/infeasibility/repair gate, the candidate oracle improves aggregate E-OSPA by 4.399%, below the 7% gate; only 5/18 states contain an admissible action with at least 5% gain. Monitoring rather than hard-gating repair on the two fixed reference-continuation steps raises the oracle to only 5.724% and 7/18 states, still below the gate. Critic training and X36 are therefore blocked; the next method must learn high-recall proposals from privileged safe graphs before value ranking. | High | E78 | This is complete six-seed development evidence, not held-out tracking performance. The repair-monitored result is a post hoc method-design sensitivity and cannot replace the frozen v1 primary result. Control metadata remains unaccounted, privileged full-space top-K capture is unevaluated, and no critic was trained. |
| C64 | On the same 18 frozen M24 states, the current-risk teacher and its leave-one-selected-edge alternatives produce 70 distinct repair-free privileged targets under the exact rolling-\(B=3\) projector, with 3–4 targets per state. All features replay exactly, the minimum within-state distinct-graph feature distance is 0.02728, and all 52 diverse targets reuse the state-matched teacher score. The frozen truth-free bank exactly matches 0/70 target graphs, exposing a proposal-space coverage gap before critic ranking. | High | E79 | Truth is used only in this offline denominator, so the targets are not deployable. Exact graph non-coverage does not establish H=3 tracking gain, top-K proposal recall, critic accuracy, X36 transfer or a communication-saving result; paired returns remain mandatory. |
| C65 | The 70 privileged targets were replayed on all 18 M24 development states with state-matched action-24 references and common random numbers. Under the frozen v1 rule that disqualifies any candidate adding a repair on the two reference-continuation steps, the oracle improves aggregate E-OSPA by 5.699%, below the 7% gate, and only 6/18 states contain an admissible action with at least 5% gain. Mean/tail/2%-byte/B3/emergency/infeasibility checks otherwise pass, with 0.094% attempted-byte deviation. Monitoring rather than hard-gating those later repairs gives 8.340% aggregate gain and 10/18 states, identifying continuation compatibility as the next design issue. | High | E80 | The v1 primary result is a failure and does not authorize distillation. The 8.340% result is a post hoc development sensitivity, not a replacement primary result. A v2 method must freeze either repair-tolerant safety-projected fallback semantics or multi-step proposals before reuse; no critic or X36 run is authorized. |
| C66 | The v2 development contract keeps the first proposed graph exact and repair-free but treats later exact safety-projection activation as legal receding-horizon adaptation. Reanalysis of the frozen v1 audit under that rule gives 8.340% aggregate E-OSPA gain, 0.001% attempted-byte deviation, tail and safety pass on every state, 10/18 states with an admissible at least 5% target, and 34 value-filtered target graphs. This passes the 7% development return gate and authorizes proposal distillation only. | High | E81 | The continuation sensitivity was inspected before v2 was frozen, so this is design-seen reanalysis and cannot support validation, generalization or paper-level effect claims. No proposal model, top-K capture, critic, unseen-seed M24 or X36 result exists yet. |
| C67 | The frozen M24 value-target dataset joins all 70 privileged graphs to their paired H=3 labels exactly and retains 34 v2 targets in 10/18 states. Per-edge features and joint-action features replay with zero numerical difference; the 34 positives span action codes 00/90/91/92 with counts 9/10/7/8, so proposal learning must represent multiple acceptable graphs rather than a single class per state. | High | E82 | Feature computation itself uses neither truth nor future outcomes, but all 18 state histories were collected under outcome-selected privileged behavior. This is a design dataset with a privileged state-distribution boundary, not deployable/held-out evidence. It authorizes only multi-solution proposal-head development; fresh truth-free online rollouts remain mandatory before critic training, M24 validation or X36. |

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
| E54 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260726_034300.md` | C40: paired local/static/learned M24 t=75–77 tracking, tail, communication, route-count and feasibility results. | strong |
| E55 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_X36_CLEAN_SCALE_T75_N1_20260726_040004.md` | C41: paired zero-shot local/static/learned X36 t=75–77 tracking, tail, communication, route-count and feasibility results. | strong |
| E56 | experiment report | `RUN/GA/dynamic_topology/DIRECTED_ROUTING_HELDOUT_T75_T77_N1_20260726_051204.md` | C42: seed-17 M24/X36 local/static/kNN continuation and gate status. | strong |
| E57 | code/report | `RUN/GA/auditDirectedRoutingGeneralization.m`, `RUN/GA/dynamic_topology/DIRECTED_ROUTING_GENERALIZATION_AUDIT_M24_X36_T75.md` | C43: reproducible standardized neighbour-distance, feature-range, cardinality-shift and receiver-coverage audit of stored M24/X36 checkpoints. | strong |
| E58 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N2_20260726_043253.md` | C44: two-seed M24 reliability weight screen with full receiver coverage. | strong |
| E59 | experiment reports | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_X36_CLEAN_SCALE_T75_N1_20260726_055125.md`, `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_X36_CLEAN_SCALE_T75_N1_20260726_070625.md` | C44: complete X36 seeds 17/7 reliability-w0.50 conditional results. | strong |
| E60 | experiment reports | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N2_20260726_061300.md`, `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_X36_CLEAN_SCALE_T75_N1_20260726_070625.md` | C45: M24 two-seed compatibility comparison and complete X36 seed-7 falsification screen. | strong |
| E61 | code/test | `common/selectDirectedTaskRoutingTeacher.m`, `RUN/GA/trainDirectedRoutingPolicy.m`, `tests/test_dynamic_topology_scenarios.m` | C46: corrected delivery expectation, complete joint-action tensors, dataset semantic validation and q=0/q=1 regression coverage. | strong |
| E62 | model report/code | `RUN/GA/dynamic_topology/CONFIDENCE_GATED_RESIDUAL_MODEL_M24_T75.md`, `common/computeScaleInvariantDirectedActionFeatures.m`, `common/fitConfidenceGatedResidualRoutingModel.m`, `common/selectConfidenceGatedResidualRoutingPolicy.m` | C47: scale-normalized joint-action model, selected-action calibration, explicit support gate and analytic fallback. | strong |
| E63 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N2_20260726_075907.md` | C48: current-runner paired M24 analytic-backbone versus retained residual-policy fallback, override/support telemetry, independent residual gate and policy-time comparison. | strong |
| E64 | code/test/audit report | `RUN/GA/dynamic_topology/DIRECTED_RELIABILITY_DYNAMICITY_AUDIT_20260726_083638.md`, `RUN/GA/auditDirectedReliabilityDynamics.m`, `common/selectRegisteredDirectedRoutingPolicy.m`, `tests/test_dynamic_topology_scenarios.m`, `docs/DIRECTED_ROUTING_ATTRIBUTION_CORRECTION_CN.md` | C44/C48/C49: all-time sender-map audit, exact fixed-index equivalence, balanced controls and boundary-excluding dynamicity diagnostics. | strong |
| E65 | experiment report/code/test | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_ORACLE_GAP_D12_HANDOVER_N1_20260726_091625.md`, `RUN/GA/runDynamicTopologyOracleGapScreen.m`, `common/selectAnalyticDirectedRoutingPolicy.m`, `common/selectRegisteredDirectedRoutingPolicy.m`, `tests/test_dynamic_topology_scenarios.m` | C50: focus-aligned strong/matched attribution gate, full physical-round-robin phase sweep, source-quality candidate result, tail, bytes, coverage and route dynamics. | strong |
| E66 | proxy report/code/test/design note | `RUN/GA/dynamic_topology/FORMATION_GATEWAY_PROXY_AUDIT_20260726_102218.md`, `RUN/GA/auditPairwiseDirectedRoutingProxy.m`, `common/selectFormationGatewayRoutingPolicy.m`, `common/selectRegisteredGatewayRoutingPolicy.m`, `tests/test_dynamic_topology_scenarios.m`, `docs/FORMATION_GATEWAY_METHOD_CN.md` | C51: training-only candidate selection key, receiver-complete constrained gateway projection, explicit formation weak-connectivity rejection and M24 snapshot proxy results. | strong |
| E67 | experiment report/code/test | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260726_104151.md`, `RUN/GA/runDynamicTopologyOracleGapScreen.m`, `common/selectRegisteredGatewayRoutingPolicy.m`, `tests/test_dynamic_topology_scenarios.m` | C52: phase-complete 15-arm M24 short closed loop, corrected link orientation, mean/tail/bytes, gateway-only dynamics, instantaneous and union formation connectivity, and attribution-gate failure. | strong |
| E68 | experiment reports/code | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260726_115612.md`, `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260726_120147.md`, `common/selectStrongFormationCycleEdges.m` | C53: fixed-index matched weight screen, exact strong-cycle projection and bridge-weight falsification. | strong |
| E69 | experiment report/code/test | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260726_120821.md`, `common/selectRootedFormationTreeEdges.m`, `common/selectConnectedFormationTreeRoutingPolicy.m`, `tests/test_dynamic_topology_scenarios.m` | C54: exact endpoint matching, previous-union strong-connectivity constraint and analytic v4 closed loop. | strong |
| E70 | experiment report/code | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260726_121146.md`, `common/selectFormationTreeRoutingTeacher.m` | C55: truth-assisted v4 closed-loop architecture signal, tail, bytes and union connectivity. | strong |
| E71 | code/test/correction | `common/convertDiagnosticEdgeMaskToPolicyAdjacency.m`, `multisensorLmb/runEventTriggeredDistributedLmbFilter.m`, `RUN/GA/generateDirectedRoutingTeacherSeries.m`, `RUN/GA/auditLearnedFormationTreeProxy.m`, `tests/test_dynamic_topology_scenarios.m`, `docs/FORMATION_GATEWAY_METHOD_CN.md` | C56: shared directed-adjacency conversion, isolated `ctxv2` dataset contract, seed-disjoint fail-closed audit, provenance boundary and explicit withdrawal of the old proxy results. | strong |
| E72 | experiment report | `RUN/GA/dynamic_topology/DYNAMIC_TOPOLOGY_CONTINUATION_X36_CLEAN_SCALE_T75_N1_20260726_114257.md` | C57: X36 v2 round-robin screen, tail, bytes and connectivity with explicit strong-control limitation. | strong |
| E73 | corrected-data diagnostic/code | `RUN/GA/dynamic_topology/STRUCTURED_TREE_CTXV2_ACTION_SPACE_DIAGNOSIS.md`, `RUN/GA/runStructuredFormationTreeCtxv2Audit.m`, `RUN/GA/auditLearnedFormationTreeProxy.m`, `common/getStructuredFormationTreeCtxv2Protocol.m`, `common/selectRootedFormationTreeEdges.m` | C58: exact current-action-space ceiling, union-constraint decomposition and fail-closed artifact decision. | strong |
| E74 | design diagnostic | `RUN/GA/dynamic_topology/STRUCTURED_TREE_CTXV2_ACTION_SPACE_DIAGNOSIS.md` | C59: rolling-\(B\) cut-constrained integer-program formulation, matched cross-edge cardinality, payload bound and five-seed proxy headroom. | medium |
| E75 | experiment audit/report/code/test | `RUN/GA/dynamic_topology/M24_COUNTERFACTUAL_CONSENSUS_TEACHER_AUDIT_SEED7_20260729.md`, `RUN/GA/auditM24CounterfactualConsensusTeacher.m`, `RUN/GA/dynamic_topology/evidence/rollout_dataset/seed7_postfusion_b01of02/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_063121.md`, `RUN/GA/dynamic_topology/evidence/rollout_dataset/seed7_postfusion_b02of02/DYNAMIC_TOPOLOGY_CONTINUATION_M24_HARD_T75_N1_20260729_063125.md`, `common/computeRollingSafeCounterfactualConsensusEdgeScores.m`, `tests/test_dynamic_topology_scenarios.m` | C60: complete mean/top-fraction \(k=1,2,3\) joint-baseline screen, exact posterior-disagreement/tracking/tail/byte readout, offline-only boundary and deterministic scorer coverage. | strong |
| E76 | versioned state artifact/preflight/code/test | `RUN/GA/dynamic_topology/datasets/rolling_safe_joint_action_state_m24_hard_t75_v3.mat` (SHA-256 `ca08e2eec2c7a3e63ceca8b3ede292cea9cdb690666f58c64be8ec4df83866c9`), `RUN/GA/dynamic_topology/datasets/ROLLING_SAFE_JOINT_ACTION_STATE_M24_HARD_T75_V3.md`, `RUN/GA/dynamic_topology/JOINT_ACTION_REPRESENTATION_PREFLIGHT_M24.md`, `RUN/GA/auditRollingSafeJointActionRepresentation.m`, `common/computeRollingSafeJointActionFeatures.m`, `common/getRollingSafeJointActionCriticProtocol.m`, `tests/test_dynamic_topology_scenarios.m` | C61: clean-generation/cache provenance, formal-reference manifest, 880-dimensional invariant representation, registered-pair non-collision, M24 observed support, synthetic cross-scale dimension and centralized-coordinator/traffic boundary. | strong |
| E77 | versioned proposal artifact/preflight/code/test | `RUN/GA/dynamic_topology/datasets/rolling_safe_joint_action_proposals_m24_hard_t75_v1.mat` (SHA-256 `71be84164cb2d46d8da4d83448e78f983a40b3760712e91078724c2620cc94e0`), `RUN/GA/dynamic_topology/datasets/ROLLING_SAFE_JOINT_ACTION_PROPOSALS_M24_HARD_T75_V1.md`, `RUN/GA/dynamic_topology/JOINT_ACTION_PROPOSAL_PREFLIGHT_M24.md`, `RUN/GA/generateRollingSafeJointActionProposalDataset.m`, `RUN/GA/auditRollingSafeJointActionProposalDataset.m`, `common/buildRollingSafeJointActionProposalBank.m`, `common/getRollingSafeJointActionCriticProtocol.m`, `tests/test_dynamic_topology_scenarios.m` | C62: clean source-state replay, frozen 28-direction truth-free bank, exact rolling-\(B=3\) projection, distinct/selectable graph accounting, repaired-reference separation, exact feature replay and broad within-state non-collision. | strong |
| E78 | complete paired-return audit/code/raw shards | `RUN/GA/dynamic_topology/JOINT_ACTION_PAIRED_RETURN_AUDIT_M24_H3.md` (raw 72-shard set SHA-256 `46c9125ae5874d03aa6c9a9e06f04a7c44288b1b9208d945c3a7838c7b616a31`), ignored `RUN/GA/dynamic_topology/joint_action_paired_return_audit_m24_h3.mat`, 72 ignored shards under `RUN/GA/dynamic_topology/returns/m24_h3`, `RUN/GA/runRollingSafeJointActionReturnShard.m`, `RUN/GA/runRollingSafeJointActionReturnBatch.m`, `RUN/GA/auditRollingSafeJointActionReturns.m` | C63: all 18 state references, 338 candidate returns, exact first-graph/common-random-number provenance, primary 4.399% oracle failure and repair-monitored 5.724% sensitivity failure. | strong |
| E79 | versioned privileged-target artifact/preflight/code/test | `RUN/GA/dynamic_topology/datasets/rolling_safe_privileged_joint_action_proposals_m24_hard_t75_v1.mat` (SHA-256 `ee1e684c1c1a6e78d21efed4f3ab0c6b32a1e3222cf9a5dc76bb8983f06ba90e`), `RUN/GA/dynamic_topology/datasets/ROLLING_SAFE_PRIVILEGED_JOINT_ACTION_PROPOSALS_M24_HARD_T75_V1.md`, `RUN/GA/dynamic_topology/PRIVILEGED_JOINT_ACTION_PROPOSAL_PREFLIGHT_M24.md`, `RUN/GA/generatePrivilegedRollingSafeJointActionProposalDataset.m`, `RUN/GA/auditPrivilegedRollingSafeJointActionProposalDataset.m`, `common/buildPrivilegedRollingSafeJointActionProposalBank.m`, `common/selectRollingSafeDiverseTeacherPolicy.m`, `common/getRollingSafeJointActionCriticProtocol.m`, `tests/test_dynamic_topology_scenarios.m` | C64: clean generation from commit `90b0cce`, 70 truth-labelled repair-free targets, cached score reuse, exact graph/feature replay and 0/70 exact coverage by the old truth-free proposal bank. | strong |
| E80 | complete privileged paired-return audit/code/raw shards | `RUN/GA/dynamic_topology/PRIVILEGED_JOINT_ACTION_PAIRED_RETURN_AUDIT_M24_H3.md` (raw 54-shard set SHA-256 `11e1076dc51a801476d06ad4381f615d41c350c36e2e5b0b9246846ab749a8bf`), ignored `RUN/GA/dynamic_topology/privileged_joint_action_paired_return_audit_m24_h3.mat`, 54 ignored shards under `RUN/GA/dynamic_topology/returns/m24_privileged_h3`, `RUN/GA/runRollingSafeJointActionReturnShard.m`, `RUN/GA/runRollingSafeJointActionReturnBatch.m`, `RUN/GA/auditRollingSafeJointActionReturns.m`, generation commit `39f2be0` | C65: all 18 references, 70 frozen privileged first-graph returns, runtime truth-use exclusion, exact graph/common-random-number provenance, primary 5.699% failure and repair-monitored 8.340% design sensitivity. | strong |
| E81 | versioned v2 development audit/code | `RUN/GA/dynamic_topology/privileged_joint_action_paired_return_audit_m24_h3_v2.mat` (SHA-256 `77f672cf11761ee0dc93a51e18f9e3d4f533d9563f8da341813a097c35d1246e`), `RUN/GA/dynamic_topology/PRIVILEGED_JOINT_ACTION_PAIRED_RETURN_AUDIT_M24_H3_V2.md`, `RUN/GA/auditPrivilegedRollingSafeJointActionReturnsV2.m`, `common/getRollingSafeJointActionCriticProtocol.m`, generation commit `e9c610b` | C66: clean-commit v2 semantics, frozen v1 source audit, 8.340% repair-aware development oracle, 34 value-filtered targets and explicit non-validation boundary. | strong |
| E82 | versioned value-target artifact/preflight/code/test | `RUN/GA/dynamic_topology/datasets/rolling_safe_privileged_value_targets_m24_hard_t75_v2.mat` (SHA-256 `cfda289bb32ecf35161786476878210b76a0ccdb3b4dfb4ee719b576e66bea63`), `RUN/GA/dynamic_topology/datasets/ROLLING_SAFE_PRIVILEGED_VALUE_TARGETS_M24_HARD_T75_V2.md`, `RUN/GA/dynamic_topology/PRIVILEGED_VALUE_TARGET_DATASET_PREFLIGHT_M24.md`, `RUN/GA/generatePrivilegedRollingSafeValueTargetDataset.m`, `RUN/GA/auditPrivilegedRollingSafeValueTargetDataset.m`, `common/getRollingSafeJointActionCriticProtocol.m`, `tests/test_dynamic_topology_scenarios.m`, generation commit `240be48` | C67: exact state/proposal/return join, zero feature replay error, 34 multi-solution value targets, truth-free feature computation and privileged state-collection boundary. | strong |

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

**Independence status: independent verifier review, not independent long-run
replication.** Two independent agents reviewed code and evidence boundaries;
one independently reran the main scenario test. They did not independently
rerun the long M24/X36 experiment reports, so the numerical result chain
remains author-run development evidence.

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
  accounting, fusion-boundary, registered directed-control, action-support,
  attribution-report and D12 callback checks;
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
- audited reliability-w0.50 over all 160 steps on M24/X36 seeds 7/17/27,
  verified its exact fixed-index clone, then ran all 11 all-physical
  round-robin phases at weights 0.50/0.70 on D12 seed 7, t=1–3;
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
- ran the complete six-action M24 seed-7 joint-baseline counterfactual
  consensus screen from the clean tracked implementation and independently
  audited its two report shards; every action reduced posterior disagreement
  but worsened realized E-OSPA, so the surrogate was rejected before model
  fitting.
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
- extended both continuations through t=77 and confirmed that posterior
  feedback does not erase the gain: E54/E55 pass the mean-tracking,
  strict-tail, communication and feasibility readouts on both scales;
- added `runDirectedRoutingHeldoutValidation.m` so missing static-prefix
  caches and paired local/static/learned screens can run incrementally per
  held-out preset/seed and leave reusable artifacts if interrupted.
- completed the seed-17 held-out continuation: the first kNN policy remains
  strong on M24 but improves X36 over local by only 1.05%, below the 5% gate;
- added and ran `auditDirectedRoutingGeneralization.m`, reproducing the
  1.165 versus 15.067/15.489 nearest-neighbour shift, 100% X36 feature-range
  violation and 18/36 versus 16/36 receiver coverage;
- corrected the one-message directed teacher from link-weight shrinkage to
  Bernoulli delivery risk and added q=0/q=1 regression coverage plus
  full-shape receiver-by-sender-by-weight risk tensors with physical
  candidates populated;
- screened registered KLA weights and reproduced a receiver-complete
  reliability-w0.50 development gain on M24 seeds 7/17 and X36 seeds 7/17;
- falsified direct compatibility replacement: moment compatibility has a
  small M24 mean benefit with worse tail/runtime and worsens X36 seed-7
  E-OSPA from 37.9153 to 40.4920;
- generated corrected M24 joint-action datasets for seeds 7/11/17, trained on
  7/17 and calibrated on 11, and retained a one-block empirical worst-error
  support gate that allows zero overrides;
- paired that residual policy against its reliability backbone on M24
  seeds 7/17 at t=75–77: all tracking/byte/route values are identical while
  the frozen report's policy time rises from 0.03 s to 17.11 s, a 17.074 s
  increment; repeated timing varied with machine load, so this is an
  overhead finding rather than an algorithmic time constant;
- audited the reliability route map over all 160 steps for M24/X36 seeds
  7/17/27. Every run has one map, zero within-window receiver changes, zero
  cross-formation messages and the same `[2,1,1,1,1,1]` local role map;
  registered fixed-index, fixed-cycle and round-robin controls and verified
  exact reliability/fixed-index routing and fusion-weight equivalence;
- separated continuation-prefix changes from within-window route changes in
  every new screen report and added an explicit dynamic-attribution gate;
- invalidated the old privileged-v4 teacher-series caches after tracing a
  sender-row/receiver-column versus receiver-row/sender-column mismatch;
  added the shared converter, isolated `ctxv2` series contract, artifact
  provenance checks, exact rooted-tree and per-step previous/current
  strong-union diagnostics, and a matched structured-tree attribution
  readout;
- reran `test_dynamic_topology_scenarios`,
  `test_dynamic_topology_screen_analysis`,
  `test_dual_threshold_event_trigger`, `git diff --check`, and the
  evidence-package lint; all passed.

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
- the first kNN confidence score is not an OOD measure and its X36 transfer
  claim is falsified by complete feature-range violation and coverage loss;
- receiver-level calibration within one shared posterior snapshot was
  rejected because receivers are dependent environment samples; only
  complete scenario/seed/time blocks are admissible calibration units;
- a posterior-compatibility score cannot replace reliability merely because
  it is closer to the KLA formula; the direct implementation is slower and
  degrades the registered X36 checkpoint.
- the apparent reliability “dynamic” gain is a fixed-index control gain:
  its reported churn came from the continuation boundary, while its sender
  map never changes inside M24/X36 evaluation windows.

Unverified:

- D12 current-task-teacher direction consistency beyond seed 7;
- the best train-selected fixed D12 graph and its held-out performance;
- method-parameter canary and full-episode behavior on seed 27; because that
  seed has already been used for geometry auditing, it is not a fresh
  scenario-held-out sample;
- a byte-matched strong directed fixed/scheduled baseline and complete
  payload-plus-control communication accounting;
- whether a residual model can make any nonzero tail-safe override over the
  frozen analytic backbone on independent scenario/seed/time blocks;
- whether no-message actions, churn limits or sliding-window connectivity add
  value once they are implemented in the same directed action space;
- the complete M24/X36 runtime-memory curve;
- whether the proposed effect-size gates are appropriately calibrated.

## Risk and Escalation

If the scenario is biased or the three-step effect does not survive held-out
seeds and longer closed-loop feedback, subsequent GNN results could be
publishable-looking but scientifically uninformative, and long Monte Carlo
runs could be wasted.
Before a paper claim or a long sweep, author review should confirm the scale
ladder, communication radius, target count, directed-message budget, tail
safety rule and Gate B/C thresholds. A domain review is also required when the
mixture-aware LMB-KLA reference is specified.

This package includes adversarial review but does not authorize a paper-facing
claim without held-out/full-episode evidence and a domain review of the KLA
reference.

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

Directed-routing model and one-/three-step checkpoint screens for E49–E55:

```bash
octave --quiet --eval "
addpath(genpath(pwd));
trainDirectedRoutingPolicy(struct('regenerateDataset',false));"

octave --quiet --eval "
addpath(genpath(pwd));
for stopTime=[75,77]
  o=struct( ...
    'maxTimeSteps',stopTime, ...
    'continuationStartTime',75, ...
    'armNames',{{'local','robust-static', ...
      'learned-directed-routing'}});
  runDynamicTopologyOracleGapScreen('m24-hard',7,o);
  runDynamicTopologyOracleGapScreen('x36-clean-scale',7,o);
end"
```

Representative output:

```text
test_dynamic_topology_scenarios passed
M24: E=15.434821 gain_static=39.492% gain_local=37.117%
      bytes_ratio=41.266% worst_static=-0.220% routes=24
X36: E=41.600155 gain_static=11.120% gain_local=6.411%
      bytes_ratio=20.834% worst_static=10.037% routes=18
M24 t75-t77: E=15.371099 gain_static=39.420% gain_local=34.480%
               bytes_ratio=30.925% worst_static=18.614%
X36 t75-t77: E=38.530987 gain_static=16.908% gain_local=10.689%
               bytes_ratio=20.272% worst_static=7.643%
```

These N=1 reports predate the current 10-seed screening/30-seed
validation-size status taxonomy; only their numerical fields are used.

Incremental held-out validation command:

```bash
octave --quiet --eval "
addpath(genpath(pwd));
runDirectedRoutingHeldoutValidation([17,27]);"
```

Only seed 17 has completed; seed 27 remains untouched. Exact commands for
E56–E63:

```bash
# E56: completed held-out kNN continuation
octave --quiet --eval "
addpath(genpath(pwd));
runDirectedRoutingHeldoutValidation(17);"

# E57: deterministic support/coverage audit
octave --quiet --eval "
addpath(genpath(pwd));
auditDirectedRoutingGeneralization();"

# E58: M24 reliability-weight screen
octave --quiet --eval "
addpath(genpath(pwd));
o=struct('maxTimeSteps',77,'continuationStartTime',75, ...
  'armNames',{{'directed-reliability-w15', ...
    'directed-reliability-w30','directed-reliability-w50', ...
    'directed-reliability-w70'}});
runDynamicTopologyOracleGapScreen('m24-hard',[7,17],o);"

# E59/E60: X36 registered w=.50 and compatibility falsification
octave --quiet --eval "
addpath(genpath(pwd));
o17=struct('maxTimeSteps',77,'continuationStartTime',75, ...
  'armNames',{{'directed-reliability-w15', ...
    'directed-reliability-w50','directed-reliability-w70'}});
runDynamicTopologyOracleGapScreen('x36-clean-scale',17,o17);
o7=struct('maxTimeSteps',77,'continuationStartTime',75, ...
  'armNames',{{'directed-reliability-w50', ...
    'directed-moment-compatibility-w50'}});
runDynamicTopologyOracleGapScreen('x36-clean-scale',7,o7);
om=struct('maxTimeSteps',77,'continuationStartTime',75, ...
  'armNames',{{'directed-reliability-w50', ...
    'directed-moment-compatibility-w50', ...
    'directed-kla-compatibility-w50'}});
runDynamicTopologyOracleGapScreen('m24-hard',[7,17],om);"

# E61/E62: regenerate direction-aware labels and fit the support-gated model
octave --quiet --eval "
addpath(genpath(pwd));
for seed=[7,11,17]
  trainDirectedRoutingPolicy(struct( ...
    'seed',seed,'regenerateDataset',true,'fitModel',false));
end
trainConfidenceGatedResidualRoutingPolicy();"

# E63: frozen residual-versus-backbone report
octave --quiet --eval "
addpath(genpath(pwd));
o=struct('maxTimeSteps',77,'continuationStartTime',75, ...
  'armNames',{{'directed-reliability-w50', ...
    'confidence-residual-directed-routing'}});
runDynamicTopologyOracleGapScreen('m24-hard',[7,17],o);"
```

Representative current outputs:

```text
held-out seed17:
  M24 kNN gain vs local = 22.62%
  X36 kNN gain vs local = 1.05% (fails 5% gate)
support audit:
  train nearest/k40 = 1.165/4.216
  X36 seed7 = 15.067/15.586, 100% range violation, 18/36 receivers
  X36 seed17 = 15.489/16.042, 100% range violation, 16/36 receivers
reliability w=.50:
  M24 seeds7/17 E-OSPA = 18.6913
  X36 seed7/17 E-OSPA = 37.9153/35.5685
residual vs backbone:
  status = residual-routing-no-incremental-gain
  E-OSPA = 18.6913/18.6913
  override = 0, support = 0.9857, exact match = 1
  policy-time overhead = 17.074 s in E63 (machine-load dependent)
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

1. Keep reliability-plus-source-quality, gateway v1, connected-tree v2,
   per-step strong-cycle v3 and every-step joint-tree v4 retired as final
   candidates. The next action space must include the fixed-index no-op,
   optional cross-formation overrides and rolling-window connectivity.
2. Add a train-frozen cross-formation role/map control under the same
   one-message-per-receiver and source-weight contract. The phase-complete
   physical round-robin is a strong scheduled template, but not an exhaustive
   frozen cross-formation optimum.
3. Extend accounting from posterior payload to beacon, scheduling, ACK/retry
   and failed-delivery control bytes before claiming total communication
   savings.
4. Implement the rolling-\(B=3\) connectivity-debt projector before making
   claims about optional cross-edge actions or window connectivity. The
   current ILP result is an oracle attainability diagnostic, not an online
   policy.
5. Treat the generic multi-neighbour `expectedDeliveryWeighting` routines as
   legacy diagnostics until their nonlinear delivery expectation is
   enumerated or bounded. Only the one-message directed teacher has the
   corrected Bernoulli-risk labels.
6. The componentwise powered-GM reference needs a stronger numerical
   comparison before it can support a paper-level density claim. The current
   receiver also excludes sources missing a label and renormalizes over the
   remaining sources rather than fusing an explicit zero-existence
   Bernoulli; a theoretical claim must disclose or replace this rule.
7. Treat corrected ctxv2 seeds 19/23/29/31/37 as development data because
   their outcomes were inspected while redesigning the action space. Allocate
   new validation and held-out seeds only after the rolling-window
   architecture, horizon and thresholds are frozen. Do not lower the gate to
   force a positive result.
8. Profile and vectorize posterior feature extraction. The current residual
   frozen report uses 17.11 policy seconds versus 0.03 seconds for the
   backbone, adding 17.074 seconds over three M24 steps while producing no
   action change. Treat exact timing as machine-load dependent.
9. Keep the joint-baseline counterfactual consensus scorer only as a negative
   offline diagnostic. Do not use its labels for imitation or value fitting:
   the complete seed-7 screen shows that improving one-step posterior
   agreement can systematically worsen tracking.
10. Build the next teacher around realized, window-cumulative tracking value
    of a complete safe joint action, with an explicit worst-node constraint.
    Its deployable model may consume only truth-free node/edge summaries and
    must select through the same rolling-\(B=3\) safety projector; truth may
    appear only in the offline development label.
11. Keep v1 rejected and use the frozen v2 contract only for development:
    its repair-aware oracle passes at 8.340% and exposes 34 value-filtered
    graphs across 10/18 states. Build a deployable truth-free multi-head edge
    proposal model, project every head through the unchanged exact rolling-
    \(B=3\) layer, and require top-K exact/value capture on at least 80% of
    target-bearing development states before fitting or evaluating a critic.
    Treat this first capture result only as an architecture diagnostic because
    all 18 histories came from outcome-selected privileged behavior. Then
    collect fresh truth-free online rollouts and repeat the frozen capture gate
    before any critic, held-out M24 or X36 claim.
12. Before any communication-saving claim, either account for centralized
    posterior-summary/control-metadata traffic or pass a frozen
    local-information ablation. X36 remains blocked until the M24 proposal
    and critic gates pass and a separate X36 protocol, compatible state
    artifact and numerical-support audit are frozen.

## Recommendation

Retire the first single-snapshot kNN transfer claim and do not replace it with
a GNN claim yet. The fixed reliability route remains a required sparse
directed baseline, not a dynamic-topology method.

The receiver-complete novelty-gated formation gateway v1 is also rejected as a
method candidate. Its local M24 t=75 proxy risk is positive, but all three
snapshot formation graphs fail the corrected weak-connectivity gate. In the
phase-complete paired t=75–77 closed loop, the gateway-only routes change and
their time union is weakly connected, yet the method gives only 1.6781%
E-OSPA gain over the strongest matched control and is not tail-safe. Do not
run this version on more M24 seeds or X36.

Corrected ctxv2 data reject every-step joint-tree v4 as the retained action
space: its exact projector itself misses the 5% block gate on 9/30 snapshots.
The next design should preserve the fixed-index intra-formation action as a
no-op, learn optional cross-formation edge values, and project them through a
rolling-\(B=3\) connectivity-debt constraint. The proxy objective should be
window-cumulative with per-step non-degradation rather than demanding 5% on
every snapshot.

The five-seed rolling-\(B=3\) ILP shows action-space headroom under matched
payload accounting, but it is privileged development evidence. Implement the
online projector and a multi-step teacher before fitting a deployable model.
Do not substitute one-step posterior consensus for that teacher: the complete
joint-baseline screen improves posterior disagreement for all six tested
actions while degrading tracking for all six. The next learning target should
rank a compact library of complete safe actions by realized window tracking
gain and worst-node safety, rather than imitate individual oracle edges or
optimize an unvalidated consensus proxy.
Only a truth-free closed-loop candidate that beats fixed-index and matched
scheduled controls on every development seed should proceed to newly
allocated validation seeds, X36 and held-out evaluation.
