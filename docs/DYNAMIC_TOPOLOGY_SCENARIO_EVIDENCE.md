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

Excluded:

- full-horizon or multi-trial D12 findings;
- M24/X36 filter runtime and memory;
- an exact arbitrary-GM density-power implementation;
- learned models or claims of performance improvement;
- a comprehensive systematic review of every distributed LMB paper.

## Risk Tier

**L3.** The author approved implementation of the scenario direction. Any
decision to claim an oracle gap, train a GNN, or change the paper story still
requires review of the full-horizon evidence.

## Claims

| ID | Claim | Confidence | Evidence IDs | Caveats |
|:--|:--|:--:|:--|:--|
| C1 | The current 4+4 driver is hard-coded for eight sensors and cannot represent the proposed multi-formation scale by changing one parameter. | High | E1, E2 | Core filter functions are more generic than the experiment driver. |
| C2 | The old 4+4 experiment does not provide positive evidence for dynamic topology; its final N50 dynamic arms were dominated or degraded. | High | E3, E4 | This is evidence about the old scene and current heuristic, not a proof that all dynamic topology is ineffective. |
| C3 | A 12/24/36 sensor ladder creates a substantially larger topology decision space while remaining a more defensible first step than jumping to 48+ sensors. | Medium | E5, E6, E7 | Runtime and memory have not yet been measured on the proposed scenes. |
| C4 | The proposed M24 formation-level ring can have a physically feasible static backbone with a 900 m communication radius along the specified pchip center trajectories. | High | E8 | The final implemented generator must reproduce this check after any parameter change. |
| C5 | Current topology infeasibility handling can violate the intended physical graph, and the bridge helper is specialized to two equal groups. | High | E9 | A replacement safe projection has not yet been implemented. |
| C6 | The current GA fusion path projects each Gaussian mixture to one Gaussian before fusion, so it cannot be the paper-level reference for mixture-aware LMB-KLA claims. | High | E10 | A density-level mixture reference will still require a documented numerical approximation. |
| C7 | An exact-oracle gate should precede GNN design because it can falsify the premise that dynamic edge choice has useful residual value. | Medium | E3, E4, E11 | The 10%/5% practical-effect thresholds are proposed preregistration values, not evidence-derived constants. |
| C8 | The current worktree provides one-call R8/D12/M24/X36 scene presets and validates trajectory bounds, separation, edge budgets, group/global connectivity, all-time static physical feasibility, D12 candidate count and handovers. | High | E12, E13 | R8's old runner remains the exact numerical-regression authority; the new R8 preset is an interface regression. |
| C9 | Dynamic topology now fails closed when the physical graph is infeasible, accepts time-varying edge loss, and reports attempted separately from delivered payload bytes. | High | E14, E13 | Control/ACK bytes are not yet added; current accounting is payload-only. |
| C10 | The implemented mixture-aware reference preserves multiple components and passes single-Gaussian and separated-identical-mixture checks, but remains a componentwise powered-GM approximation rather than exact arbitrary-mixture KLA. | High | E15, E13 | Paper-level theory must retain this approximation boundary or replace it with a stronger numerical reference. |
| C11 | The first 8-step smoke is not evidence of a dynamic-topology gain; it exposed metric saturation under the inherited 5 m OSPA cutoff and confirmed that a myopic one-step oracle is not a closed-loop upper bound. | High | E16 | The corrected smoke covers only the pre-handover window and one seed. |

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
- ran `test_dynamic_topology_scenarios`, which passed all preset, safety,
  accounting, fusion-boundary and D12 callback checks;
- reproduced M24 limits from the implemented generator: 24 sensors, 30 static
  edges, maximum speed 11.45 m/s, acceleration 0.86 m/s² and target speed
  13.80 m/s;
- reproduced X36 limits: 36 sensors, 45 static edges, maximum speed
  6.41 m/s, acceleration 0.41 m/s² and target speed 13.67 m/s;
- ran an 8-step six-arm smoke with attempted-byte mismatch below 1% for the
  consensus oracle comparison after adding a per-step byte feasibility gate.

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

Unverified:

- full-horizon FoV handover benefit and target visibility balance;
- D12 results beyond one seed and beyond the first eight steps;
- M24/X36 filtering runtime and memory;
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
```

Core inspection commands:

```bash
nl -ba RUN/GA/runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare.m | sed -n '416,478p;1744,1795p'
nl -ba multisensorLmb/runEventTriggeredDistributedLmbFilter.m | sed -n '968,1007p;1124,1141p'
nl -ba multisensorLmb/gaLmbTrackMerging.m | sed -n '1,100p;150,164p'
nl -ba docs/EFFECTIVE_KLA_GRAPH_VALIDATION_STATUS_CN.md | sed -n '121,174p'
nl -ba docs/DUAL_THRESHOLD_EVENT_TRIGGER_RESEARCH_CN.md | sed -n '235,253p'
git diff --check
```

Evidence-package lint:

```bash
python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py docs/DYNAMIC_TOPOLOGY_SCENARIO_EVIDENCE.md
```

## Open Issues

1. The componentwise powered-GM reference needs a stronger numerical
   comparison before it can support a paper-level density claim.
2. The D12 one-step consensus and truth objectives need full-window evidence;
   neither may be described as a global upper bound.
3. The implemented generator must repeat route, speed, acceleration and
   static-backbone checks after any trajectory change.
4. The main FoV radius and target paths may need adjustment after full-window
   coverage and tracking results.
5. Runtime may require sparse diagnostics before X36.
6. With the author's current limit, exploratory execution must not exceed
   three paired trials.

## Recommendation

Proceed with at most three paired full-horizon D12 trials as an exploratory
oracle-gap screen. Do not design or train the GNN yet. The infrastructure gates
are now implemented (C8–C10), but the corrected smoke is deliberately too
short and too small to establish a research direction (C11).

If the full handover window still shows less than the practical 10% consensus
or 5% tracking gap under matched attempted bytes, treat that as a scene/method
premise failure and stop before learning. If a residual gap appears, first
measure how much the posterior-discrepancy heuristic captures.
