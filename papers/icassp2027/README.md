# Independent ICASSP 2027 manuscript

This directory starts a new paper from the official 2027 template archive.
It does not reuse the previous ICASSP manuscript or the Adaptive-KLA text.

- `main.tex` / `main.pdf`: English research draft.
- `MAINLINE_PROGRESS_CN.md`: current direction, paired result tables and next decision.
- `references.bib`: metadata retrieved from DOI/Crossref records.
- `TEMPLATE_SOURCE.md`: template origin and official requirements.
- `official-template/`: untouched style sources and the complete downloaded archive.

Build from this directory:

```sh
tectonic --keep-logs main.tex
```

The working title is **Delivery-Aware Sparse Causal Routing for Distributed
Multi-Object Tracking**. Jinhao Chen is first author; Tianyu Wo is corresponding
author. Both affiliations are Beihang University, as confirmed by the author.
No funding and no conflicts of interest were declared.
The supplied English names are authoritative. Chinese-character expansions
in the message of commit `1185a5d` were not author-confirmed and should not
be used; they do not appear in the manuscript or its author block.

The canonical progress document remains
[the existing Lark document](https://jjp48fb03jzs.jp.larksuite.com/docx/HcFFdtKIRovhHLxKrx5jVpiBpJh).
The independent paper is maintained on `codex/icassp2027-sparse-causal-routing`.

The manuscript contains the implemented sparse causal backbone, a scheduled
connectivity/count proof, a conditional finite-round geometric-pooling bound,
and complete three-arm M24/X36 development results. The X36 self-weight
fallback ablation is also complete: set error and consistency improve over
sparse renormalization, but conditional RMSE worsens by 4.836%, failing the
predefined M24 follow-up gate. The draft includes the V279 count-error
budget and common-finite-cell RMSE comparison. Detailed V280 opportunity
and V282 weak-input diagnostics remain in the progress notes and experiment
records rather than the compact manuscript. Two Python-rendered vector
figures expose causal tree repair, directed delivery and the payload/set-error
tradeoff. The method figure now appears on page 2 and distinguishes a failed
physical branch from a lost packet after scheduling. The result figure
now includes fixed-count localization ceilings from the saved count-budget
data, making the remaining target-set recovery problem explicit. Figure sources,
data and regeneration instructions are in `figures/README.md`.
No multiseed significance, exact arbitrary-GM powers, globally minimum graph,
loss-resilient equal-weight consensus or end-to-end bandwidth saving is claimed.

Before submission: locate the target-existence bottleneck and establish a
jointly beneficial method on both scales; freeze independent seeds and the
second scene style; assess control traffic;
expand the routing-specific related-work comparison and establish novelty;
finalize the author-reviewed manuscript and any required AI-use disclosure.
The derived perturbation bound is elementary and scoped; it is not by itself
evidence of a novel estimation-optimal policy.

V280 remains a geometry/packet-opportunity diagnostic, not an actual-label
propagation experiment; its readout is retained in the mainline progress note.
It reuses opened cases and changes no filter or sensing parameter. The V278
background process finished normally; no automatic M24 extension was started.

V281 is complete on three cached M24 anchors: local-posterior MAP counts are
already about six before the final fusion, whose immediate overlap and
pruning mass losses are small. That finding motivated the subsequent local
update and propagation trace, keeping existence pooling distinct. The detailed
record is not a new policy result, a full-episode causal attribution or X36
evidence; it does not change the paper's performance table.

V282 and V283 are now complete on the unchanged X36 prefix, steps 1--40.
All output metric cells match the stored reference prefix. Late weak pools
persist after observation-opportunity flags have spread: never-informed
inputs supply 0.937% of negative weighted log-odds magnitude at steps 36--40,
and 3,117/3,227 weak pools have no input with r >= 0.9. These descriptive
statistics are not a new tracking result and do not exclude a lasting
cold-start effect. The next design targets useful evidence strength and age
over repeated hops, not geometry-only input exclusion or output thresholds.
See `RUN/GA/dynamic_topology/V283_OBSERVATION_EVIDENCE_FINDING.md` from the
repository root. No additional filter was launched with that diagnostic.

The paper now distinguishes GM-PHD CA-GCI from multi-view LMB prior art and
from the repository's own absent-label FoV censor. Li et al. (FUSION 2018)
and Wang et al. (Signal Processing 2018) were added with verified DOI metadata;
the G. Li et al. reference uses its 2020 journal record. Full algorithmic
reproduction and a faithful multi-view baseline are still open.

The routing-related comparison now includes Ramachandran et al., IEEE TCNS
2021, DOI 10.1109/TCNS.2021.3059794. Topology reconfiguration, small edit count
and joint topology/weight design already have prior art. Our remaining
novelty burden is the useful interaction of a fixed sparse budget, directed
delivery and finite-round LMB target-set recovery, not dynamic topology alone.

V284--V286 now narrow the design requirement to joint existence retention and
reliable spatial updates. The paired prefix and saved-output source analyses
remain experiment records, not new best-table rows. Global same-label output
headroom does not establish an available, accurate source selector; replacing
component covariance with full-GM output risk still harms some formations in
the final-snapshot check. The manuscript and mainline note state this design
boundary without presenting oracle figures as tracking improvements.
The new two-panel source diagnostic is retained with the V286 experiment,
outside the two manuscript figures. No additional filter was launched with
those source-only diagnostics.

V287 adds a source-correspondence restriction to the same saved X36 queries:
the lag-one global oracle changes from 11.083 m to 22.350 m when the source's
own geometric assignment must agree with the receiver query (receiver 34.787 m).
These truth-based pooled diagnostics neither change the official metrics nor
prove identity errors caused the tracking gap. The next comparison must separate
label-matched fusion from routing effects. The paper now cites Li et al. (TSP
2019) and Gao et al. (TAES 2022), explicitly states that current arms do not
perform spatial label matching, and keeps the performance table unchanged.
Their metadata and high-level claims are checked; the 2019 full implementation
has not been obtained or reproduced. The earlier 2016 GCI-LSM text is not a
drop-in substitute for unequal-FoV unmatched-track handling.
The V287 figure remains in the experiment record; the approved Lark method
board is preserved. See `RUN/GA/dynamic_topology/LABEL_MATCHING_BASELINE_NOTES.md`
for the bounded two-by-two fusion/routing comparison design.

V288 now starts that comparison by isolating the fusion objective **before**
geometric label reassignment. The shared-label LMB-MIL receiver follows the
accessible Gao et al. author version (arXiv:1911.01083v1, Proposition 3), with
explicit zero-existence extension and diagnosed eight-component GM reduction.
It preserves the V242 route and is not the complete different-FoV/matching
algorithm. Formula checks and two-step integration passed; the 40-step X36
candidate completed from source-freeze commit `f67a1bb` and failed its joint
screen. This version is not extended to M24 or a full episode. Detailed
candidate results remain in experiment records, not the main best table.
The numerical main table is unchanged. The draft distinguishes the three
factors, adds the verified 2020 MIL reference, and still has five pages with
the complete conclusion on page 4 and declarations/references on page 5.
See `RUN/GA/dynamic_topology/V288_RUN_STATUS.md` for the current run record.
An analytic fusion-rule schematic is retained in the baseline notes, not
introduced as tracking evidence or substituted for the approved Lark board.

V290 now isolates the local FoV-boundary approximation using published
Gaussian splitting, with unchanged KLA and V242 routing. The two-step
integration completed; it does not demonstrate a tracking improvement.
The single 40-step X36 screen completed from source checkpoint `702f84d`
and failed the joint screen: set/localization/count error worsened and
posterior bytes increased. It remains default-off, with no M24/full extension.
Its source, completed tradeoff and execution command are recorded in
`RUN/GA/dynamic_topology/V290_FOV_SPLITTING_DESIGN.md`. Neither that baseline
nor the new explanatory figure is a newly validated routing contribution.

V291 next isolates the conditional spatial-pooling operator relative to MIL,
retaining its arithmetic existence, zero extension and conditional weights.
It is an explicit alternative objective, not standard LMB-KLA, LMB-MIL,
a reproduced Uney estimator or a new routing method. The manuscript now cites
Uney et al. (TAES 2019) for prior cardinality/localization decoupling and
explicitly excludes local FoV splitting from the reported routing arms.
The new analytic operator figure stays in the design record, not the main
paper or approved Lark board. No candidate row has been added to the main table.
