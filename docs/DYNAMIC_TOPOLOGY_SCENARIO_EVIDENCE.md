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
  shortlisted fixed-graph control, and one M24 scale/action-signal snapshot.

Excluded:

- an exhaustive performance sweep over all 48 fixed D12 graphs;
- a multi-seed held-out validation of the current-task-risk teacher;
- a complete M24/X36 filter runtime and memory scaling curve;
- an exact arbitrary-GM density-power implementation;
- learned models or claims of performance improvement;
- a comprehensive systematic review of every distributed LMB paper.

## Risk Tier

**L3.** The old one-step teacher has a confirmed stop finding. The current
task-risk teacher has positive single-seed D12 closed-loop evidence and an M24
action-signal snapshot, but no held-out learned policy or multi-seed effect
estimate. The current evidence authorizes label-pipeline development, not GNN
performance claims or a paper claim of dynamic-topology superiority.

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
| E13 | test | `tests/test_dynamic_topology_scenarios.m`; command in Verification Record | C8–C10: all seven presets validate; D12 has 48 fourteen-edge candidates; scheduled births, time-varying loss, attempted bytes, fail-closed topology, KLA boundary cases and exact-callback smoke pass. | strong |
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

Unverified:

- D12 current-task-teacher direction consistency beyond seed 7;
- the best train-selected fixed D12 graph and its held-out performance;
- whether a locally observable learned policy can imitate the privileged
  current-task-risk labels;
- M24 closed-loop tracking gains and the complete M24/X36 runtime-memory curve;
- whether the proposed effect-size gates are appropriately calibrated.

## Risk and Escalation

If the scenario is biased or computationally infeasible, subsequent GNN results could be publishable-looking but scientifically uninformative, and long Monte Carlo runs could be wasted. Author review is required for the scale ladder, communication radius, target count, and Gate B/C thresholds. A domain review is also required when the mixture-aware LMB-KLA reference is specified.

No paper claim, method choice, or long experiment should treat this draft as approved before that review.

## Reproducibility

Repository, audited source baseline, and scenario checkpoint:

```text
/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/learned-dynamic-topology-scenarios
audited source baseline: 282ca8180510315424dbb488ce7cfd80e624115f
scenario checkpoint: 2c75193
three-trial implementation checkpoint: b63fd2b
```

Core inspection commands:

```bash
nl -ba RUN/GA/runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare.m | sed -n '416,478p;1744,1795p'
nl -ba multisensorLmb/runEventTriggeredDistributedLmbFilter.m | sed -n '968,1007p;1124,1141p'
nl -ba multisensorLmb/gaLmbTrackMerging.m | sed -n '1,100p;150,164p'
nl -ba docs/EFFECTIVE_KLA_GRAPH_VALIDATION_STATUS_CN.md | sed -n '121,174p'
nl -ba docs/DUAL_THRESHOLD_EVENT_TRIGGER_RESEARCH_CN.md | sed -n '235,253p'
octave --quiet --eval "setPath; addpath('tests'); addpath(fullfile('RUN','GA')); test_dynamic_topology_screen_analysis"
git diff --check
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
5. Use cached sparse posterior snapshots for M24 label generation; a 75-step
   behavior trajectory already costs 559.37 s on the current machine.

## Recommendation

Retire the old consensus/truth-composite teacher and the redundant open-loop
look-ahead labels. Keep pure current task risk as the privileged supervision
candidate: it has a positive D12 closed-loop screening result against both
posterior-discrepancy and an in-sample stronger fixed graph (C20/C21), and it
retains substantial action separation at M24 scale (C22).

The next stage is label-pipeline and imitation validation, not a paper-level
GNN experiment. Select the strongest fixed D12 graph only on training seeds,
train a policy using locally available inputs to imitate current-task-risk
rankings, freeze it, and then run paired held-out D12 tests. M24 closed-loop
evaluation is justified only after that frozen policy passes the D12 gate.
