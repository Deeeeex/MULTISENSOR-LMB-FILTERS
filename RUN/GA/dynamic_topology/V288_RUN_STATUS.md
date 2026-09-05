# V288 shared-label MIL: implementation and running prefix

## Question

Does a known arithmetic LMB receiver improve the joint recovery/localization
tradeoff under the unchanged sparse route? Separate fusion-rule, label-matching
and routing effects before claiming a new method.

## Scope

Shared-label MIL implementation, formula checks, a completed two-step X36
integration, one running 40-step development candidate, an analytic schematic,
and scoped manuscript/Lark updates. No complete-prefix result yet; no new
best-method row, M24 result, multiseed validation or paper submission.

## Risk Tier

L2 exploratory research. This draft record is self-check only; no externally
validated scientific-performance claim or final method selection is authorized.

## Claims

| ID | Claim | Evidence | Caveat |
| --- | --- | --- | --- |
| C1 | Shared-label reduced-GM MIL is implemented with one source-weight vector and explicit zero-existence missing labels. | E1, E2 | Not KLA, label matching or full different-FoV MIL; eight-component reduction is approximate. |
| C2 | The two-step integration completes with the same attempted/delivered edge masks. | E3 | Integration evidence only; E-OSPA is 139.697349 -> 139.270653, but RMSE and bytes increase. |
| C3 | A frozen 40-step candidate is running from the committed source, with the old filter arm reused. | E4 | No result or screen outcome exists yet. |
| C4 | The paper and Lark now distinguish fusion, correspondence and routing; the explanatory plot is analytic. | E5, E6 | Does not increase evidence for a successful tracking method. |

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
  The exact 40-step command below is running in session 63055. Its log reports
  `Filter starting step 11/40 at 2026-09-06 07:00:10`. Runtime source files
  have not been changed during the run. The growing log is not a completed
  experiment artifact.
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
The completed screen must jointly assess accuracy, cardinality, coverage,
communication and formation tails before any extension.

## Reproducibility

Run from the isolated ICASSP worktree, with Octave 11.1.0:

```sh
mkdir -p RUN/GA/dynamic_topology/evidence/tracking_aligned_v288/x36_prefix40_seed1301
set -o pipefail
/opt/homebrew/bin/octave --no-gui --quiet --eval "addpath(genpath(pwd)); o=struct('baselineTracePath','RUN/GA/dynamic_topology/evidence/tracking_aligned_v282/x36_prefix40_seed1301/EXISTENCE_STAGE_TRACE_V282.mat','maximumTime',40,'outputRoot','RUN/GA/dynamic_topology/evidence/tracking_aligned_v288/x36_prefix40_seed1301'); [p,r]=runCommonLabelLmbMilV288(o); disp(p);" 2>&1 | tee RUN/GA/dynamic_topology/evidence/tracking_aligned_v288/x36_prefix40_seed1301/run.log
```

Do not repeat that command while session 63055 is alive. Observe with
`tail -f RUN/GA/dynamic_topology/evidence/tracking_aligned_v288/x36_prefix40_seed1301/run.log`.
The original complete 160-step observations are generated before the 40-step
crop. Saved raw results are reused on subsequent analysis calls. MAT files and
logs remain local/ignored; numerical reports and figure data are tracked.

## Open Issues

The 40-step outcome, truncation burden and costs are pending. A zero-extended
represented-posterior MIL baseline is not the full exclusive-FoV construction.
Neither label matching nor causal spatial-quality routing is implemented here.
The main M24/X36 joint goal is still unmet; the approved main board is unchanged.

## Recommendation

Keep C1--C2 as implementation evidence and wait for E4 to finish; do not call
MIL a successful candidate. Analyze the joint screen once, without changing
the frozen weights, reduction cap or source semantics for this opened seed.
If it fails, retain the result only in experiment records and distinguish its
limitations from those of the full published MIL method.
