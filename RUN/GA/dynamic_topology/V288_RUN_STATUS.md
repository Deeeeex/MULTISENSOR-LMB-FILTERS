# V288 shared-label MIL: completed prefix and method decision

## Question

Does a known arithmetic LMB receiver improve the joint recovery/localization
tradeoff under the unchanged sparse route? Separate fusion-rule, label-matching
and routing effects before claiming a new method.

## Scope

Shared-label MIL implementation, formula checks, a completed two-step X36
integration, the completed 40-step development candidate, an analytic schematic,
and scoped manuscript/document updates. The joint screen failed. No new
best-method row, M24 result, multiseed validation or paper submission.

## Risk Tier

L2 exploratory research. This draft record is self-check only; no externally
validated scientific-performance claim or final method selection is authorized.

## Claims

| ID | Claim | Evidence | Caveat |
| --- | --- | --- | --- |
| C1 | Shared-label reduced-GM MIL is implemented with one source-weight vector and explicit zero-existence missing labels. | E1, E2 | Not KLA, label matching or full different-FoV MIL; eight-component reduction is approximate. |
| C2 | The two-step integration completes with the same attempted/delivered edge masks. | E3 | Integration evidence only; E-OSPA is 139.697349 -> 139.270653, but RMSE and bytes increase. |
| C3 | The frozen 40-step candidate completed from the committed source, with the old filter arm reused and identical route masks. | E4, E8 | One opened prefix, not a full episode or independent validation. |
| C4 | The paper and Lark now distinguish fusion, correspondence and routing; the explanatory plot is analytic. | E5, E6 | Does not increase evidence for a successful tracking method. |
| C5 | MIL modestly improves set error, count error and disagreement, but worsens conditional RMSE and attempted bytes, failing the frozen joint screen. | E8, E9 | Reduced-GM shared-label MIL only; not a rejection of complete different-FoV MIL. |

## Evidence Ledger

- E1: [Gao et al. author version](https://arxiv.org/html/1911.01083v1),
  Proposition 3, (26)--(28), plus Sections IV and V-B read directly.
  [arXiv metadata](https://arxiv.org/abs/1911.01083) links related journal
  DOI `10.1109/TSP.2020.3028496`. Command
  `curl --fail --silent --show-error --location 'https://api.crossref.org/works/10.1109%2FTSP.2020.3028496/transform/application/x-bibtex'`
  returned the Gao/Battistelli/Chisci TSP 68:5855--5868 (2020) record.
- E2: `octave --quiet --eval "addpath(genpath(pwd)); checkCommonLabelLmbMilV288();"`
  returned `V288 formula self-check PASS: weighted existence/GM, absent labels, reduction, override guard, default KLA.`
  Runtime implementation is `multisensorLmb/fuseCommonLabelLmbMil.m` and the
  default-off dispatch in `fuseLmbPosteriorsByLabel.m`.
- E3: The command under Reproducibility with `maximumTime=2` and directory
  `x36_prefix2_integration_seed1301` exited 0 (session 77843), with filter time
  25.9 s, 92/91 attempted/delivered messages, edge-mask differences 0/0,
  attempted bytes 677584 -> 964696 and conditional RMSE 11.831774 -> 13.737502.
  Source was an uncommitted working implementation based on `d5fa9e4`, frozen
  in E4. The two zero-extension counters were added after this integration;
  that old MAT does not contain them. Numerical Markdown/CSV remain available.
- E4: Commit `f67a1bb`, `Add an isolated shared-label MIL fusion control`, was
  pushed successfully: `d5fa9e4..f67a1bb codex/icassp2027-sparse-causal-routing`.
  The exact 40-step command below completed in session 63055 with exit 0.
  The raw file records source `f67a1bbc5f19c56fe4005749888b4ef964437230` and
  filter time 2205.5 s; subsequent metric aggregation is not included in
  that filter time. Runtime source files were unchanged throughout the run.
- E5: `tectonic --keep-logs main.tex` in `papers/icassp2027` exited 0; PDF
  text/page inspection reports five pages, complete conclusion on page 4,
  declarations and 13 references on page 5. Python-rendered changed pages
  1, 4 and 5 were visually inspected without overlap or clipping. Known
  underfull boxes and repeated BibTeX rerun warnings remain; no missing
  citation warning. Lark `block_replace` updated only four text paragraphs;
  `docs +fetch --doc HcFFdtKIRovhHLxKrx5jVpiBpJh --scope keyword --keyword '融合、对应和传递|融合准则与标签对应|目前卡点|下一道决策' --detail with-ids --as user`
  returned revision 1260 with the intended text. No board or result-table edit.
- E6: `/Users/dex/miniconda3/bin/python3 RUN/GA/plot_fusion_rule_schematic_v288.py RUN/GA/dynamic_topology/evidence/tracking_aligned_v288/analytic_fusion_schematic`
  returned `Analytic figure exported: eta=0.011108997, KLA r=0.042545438, MIL r=0.800`.
  Its two CSVs, editable SVG/PDF, 600-dpi PNG and parameter manifest are retained;
  PNG visually checked. No truth, empirical error, seed or uncertainty bar is
  represented in these exact one-dimensional curves.
- E7: `git diff -- papers/icassp2027/MAINLINE_PROGRESS_CN.md` was inspected
  after a consistency-only update: the TL;DR, method paragraph, related work
  and next-action rows now agree with the actual fixed-label MIL control.
  The full-episode numerical tables and original two method/result figures
  are unchanged. This is documentation synchronization, not new performance
  evidence; the background receiver remains frozen at `f67a1bb`.
- E8: The E4 command returned `V288 prefix 40: E-OSPA 135.180030 -> 132.617637
  (+1.896% gain), RMSE 8.425317 -> 19.735674, bytes ratio 3.2474, screen
  evaluated 1 / passed 0.` The completed report and three numerical CSVs
  are in `evidence/tracking_aligned_v288/x36_prefix40_seed1301/` relative to
  this record. Raw/result MAT files are retained locally. Attempted/delivered
  mask differences are 0/0, with 1840/1766 messages in both arms.
- E9: `V288_SENSOR_METRICS.csv` has 36 paired rows: all 36 E-OSPA differences
  are negative and all 36 RMSE differences are positive. Formation RMSE is
  the mean of its six sensor rows, valid here because every sensor has all
  40 finite RMSE cells. `V288_TIME_SERIES.csv` split into equal halves gives
  reference/candidate RMSE 8.960806/15.191155 m at steps 1--20 and
  7.889829/24.280192 m at steps 21--40. These are descriptive within-episode
  summaries, not independent replicates or evidence about matched identities.
- E10: Three scoped Lark paragraph replacements completed successfully;
  `lark-cli docs +fetch --doc HcFFdtKIRovhHLxKrx5jVpiBpJh --scope keyword --keyword '目前卡点|下一步：把融合|下一道决策' --detail with-ids --as user`
  returned revision 1263 with the completed-control status and the next
  unequal-FoV/byte-budget decision. No numerical candidate row, board or
  new document was added. The local mainline note uses the same status.

## Completed result and interpretation

| Quantity | V242 KLA | Reduced-GM MIL | Change |
| --- | ---: | ---: | ---: |
| E-OSPA, m | 135.180030 | 132.617637 | 1.896% lower |
| Absolute count error | 19.501389 | 18.631944 | 4.458% lower |
| Conditional matched RMSE, m | 8.425317 | 19.735674 | 134.242% higher |
| Representative disagreement, entire prefix, m | 144.669699 | 140.162516 | 3.115% lower |
| Attempted posterior payload, decimal MB | 18.435344 | 59.867264 | 3.2474 times |
| Finite RMSE coverage | 100% | 100% | unchanged |

The RMSE deterioration is not caused by losing finite sensor--time cells:
all 1440 cells are finite in both arms. Their matched target sets can still
differ, so this does not isolate the error of identical target identities.
All six formations improve E-OSPA, but their RMSE increases range from
78.324% to 181.395%. The later-half RMSE gap is larger, not just an initial
transient. None of these single-episode summaries establishes generalization.

Equal routing does not mean equal byte cost. The receiver keeps richer
posteriors that affect subsequent packet sizes; even after the common
eight-component cap, attempted payload is 59.867 MB. Spatial reduction
truncates 22342/34560 label pools (64.647%); mean removed conditional mass is
0.074527607 and the maximum is 0.663394592. The experiment therefore cannot
attribute the spatial error solely to exact MIL's fusion objective.
The missing-label zero-extension counters are both zero in the MIL arm:
its declared missing-label branch is not activated on this prefix. This
does not make the represented-posterior pruning histories of MIL and KLA
identical, nor constitute a test of exclusive-FoV fusion.

## Verification Record

Self-check only. The first integration request was rejected because its MIL
missing-label semantics differed from the registered KLA boundary. A narrowly
identified MIL control was added to the V242 development validator, preserving
route, message budget, observable context and no-truth requirements. The
corrected integration completed. No separate verifier or held-out run.
`python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py RUN/GA/dynamic_topology/V288_RUN_STATUS.md`
returned `PASS: RUN/GA/dynamic_topology/V288_RUN_STATUS.md`; that format check
does not independently verify the method or its eventual performance.

## Risk and Escalation

A fusion improvement is not a routing improvement. MIL may preserve inaccurate
or false tracks and increases the number of transmitted mixture components.
Changing missing-label semantics is explicit in this receiver-family control;
do not attribute all differences solely to removing the overlap factor.
The completed screen rejects an extension of this version because accuracy
and actual byte cost deteriorate. A capped-packet graph is not a byte-budget
guarantee when the posterior representation changes.

## Reproducibility

Run from the isolated ICASSP worktree, with Octave 11.1.0:

```sh
mkdir -p RUN/GA/dynamic_topology/evidence/tracking_aligned_v288/x36_prefix40_seed1301
set -o pipefail
/opt/homebrew/bin/octave --no-gui --quiet --eval "addpath(genpath(pwd)); o=struct('baselineTracePath','RUN/GA/dynamic_topology/evidence/tracking_aligned_v282/x36_prefix40_seed1301/EXISTENCE_STAGE_TRACE_V282.mat','maximumTime',40,'outputRoot','RUN/GA/dynamic_topology/evidence/tracking_aligned_v288/x36_prefix40_seed1301'); [p,r]=runCommonLabelLmbMilV288(o); disp(p);" 2>&1 | tee RUN/GA/dynamic_topology/evidence/tracking_aligned_v288/x36_prefix40_seed1301/run.log
```

Session 63055 has exited 0. The run must not be restarted to fill an observation
timeout or improve the opened-seed result. The same command reuses its saved
raw file for metric regeneration; it need not rerun filtering.
The original complete 160-step observations are generated before the 40-step
crop. Saved raw results are reused on subsequent analysis calls. MAT files and
logs remain local/ignored; numerical reports and figure data are tracked.

## Open Issues

A zero-extended represented-posterior MIL baseline is not the full
exclusive-FoV construction. The separate effects of objective, recursive GM
reduction and posterior history are not causally isolated by this control.
Neither label matching nor causal spatial-quality routing is implemented here.
The main M24/X36 joint goal is still unmet; the approved main board is unchanged.

## Recommendation

Stop this version at the completed prefix: no full episode, M24 extension,
same-fusion fixed-route extension, or weight/truncation sweep on this opened
seed. Keep C5 in experiment records, not the main best-method table.

Retain the sparse routing reference without calling it jointly optimal.
Before selecting a new routing method, establish a faithful unequal-FoV
fusion comparison with explicit treatment of exclusive evidence, reliable
spatial support and real posterior bytes. Do not add geometric relabeling
merely because this MIL control failed: the shared birth-label model has
not been disproved. Only same-fusion routing comparisons can establish a
routing contribution; existing MIL/matching methods are baselines, not novelty.
