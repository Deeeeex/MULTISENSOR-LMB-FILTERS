# Research handoff: sparse causal LMB routing

## Question

Which single research direction is supported by the completed M24/X36 results,
and what can be written now in an independent ICASSP 2027 draft?

## Scope

V248 M24 and V274 X36 full episodes; V277 packet-only replay; implemented
V240/V242 routing; completed V278 single-factor receiver ablation; official ICASSP kit.
V279 adds a post-hoc count-error budget from the same saved paired episodes.
V280 adds geometric observation and ideal-retention packet-path opportunity.
V281 adds a completed one-round reference replay at three saved M24 anchors.
V282 adds the completed unchanged-reference X36 trace, steps 1--40.
V283 separates untouched states from historical observation-opportunity paths
in that same trace, without a new filter run or candidate evaluation.
Old full/light equivalence and earlier fusion-code results are excluded.

## Risk Tier

L2: reproducible internal research draft. No submission or publication action.

## Claims

| ID | Claim | Evidence | Caveat |
|:--|:--|:--|:--|
| C1 | Sparse repair reduces attempted posterior bytes in both executed cases. | E1, E2 | One seed per scale; control traffic excluded. |
| C2 | X36 RMSE gain coexists with cardinality and consistency deterioration. | E2 | Conditional RMSE is not complete-set accuracy. |
| C3 | Planned balance and strong connectivity do not survive every packet realization. | E3, E4 | Packet-level replay, not label-wise state propagation. |
| C4 | Self fallback preserves surviving-neighbor planned weights but is not uniformly better for tracking. | E4, E7 | The completed X36 ablation has a precision-consistency tradeoff, not a cross-scale validation. |
| C5 | With per-cell estimate counts fixed, sparse-arm mean E-OSPA can improve by at most 0.279 m on M24 and 1.311 m on X36 through localization alone. | E6 | Algebraic post-hoc bound; X36 count-sign ambiguity is retained. Not an achieved policy gain. |
| C6 | On X36 sensor-time cells with finite RMSE in both arms, sparse-vs-fixed RMSE gain is 38.431%. | E6 | Same cells do not imply the same matched target identities. |
| C7 | X36 self fallback improves E-OSPA by 0.175% and focus consistency by 0.865% versus V242, but worsens conditional RMSE by 4.836%. | E7 | Single opened seed; fails the frozen 1% RMSE tolerance for an M24 follow-up; worst formation RMSE change is -37.361%. |
| C8 | Global geometric blackout occupies 2.773%/2.734% of active target-time on M24/X36; sparse eight-step geometric-path coverage is 81.261%/63.183%. | E8 | Perfect detection and indefinite retention assumed for paths. Not actual labels, posterior recall or causal attribution of tracking error. |
| C9 | At saved M24 anchors 70/84/151, local-posterior MAP readout is already about six; the immediate spatial term removes mean existence mass 0.119/0.225/0.099. | E11 | Current reference replay of three post-local-update states, not a full-episode causal decomposition or X36 result. Labels and existence mass are not true-target recall. |
| C10 | At 27,939 zero-component-mean-pD X36 label stages, maximum absolute local existence change is 2.22e-16; step-40 local/output MAP counts are 5.806/5.861. | E13 | Unchanged 40-step opened-seed prefix. Does not prove the component-mean sensing approximation accurate or exclude earlier cumulative loss. |
| C11 | At X36 steps 36--40, never-informed inputs supply 0.937% of negative weighted log-odds magnitude; 3,117/3,227 weak pre-spatial pools contain no active input with r >= 0.9. | E14 | Opportunity lineage includes missed detections, not confirmed detections or retained information strength. Descriptive levels are not output thresholds; lasting cold-start effects remain possible. |
| C12 | The repository's absent-label FoV censor is not an implementation of the cited complete multi-view PHD/LMB methods; information-based label weighting has prior art. | E4, E15 | Metadata and scoped source claims checked; full LMB algorithm reproduction and a faithful baseline remain open. |

## Evidence Ledger

| ID | Artifact / command | Representative evidence |
|:--|:--|:--|
| E1 | `RUN/GA/dynamic_topology/evidence/tracking_aligned_v248/m24_temporal_task_coupled_formation_braid_seed1301/TEMPORAL_TASK_COUPLED_FORMATION_BRAID_V248_FULL_EPISODE.md` | Sparse: 122.462 E-OSPA, 12.183 RMSE, 36,675,624 B. |
| E2 | `RUN/GA/dynamic_topology/evidence/tracking_aligned_v274/x36_minimum_backbone_seed1301/` | Sparse: 132.192 E-OSPA, 19.329 RMSE, 60,090,416 B; cardinality error 18.597396 vs 18.455729. |
| E3 | `octave --no-gui --quiet --eval "addpath(genpath(pwd)); analyzeRealizedKlaBalanceV277();"` | Sparse packet-strong steps: 41/160 M24, 18/160 X36; non-double steps: 122/160, 143/160. |
| E4 | `multisensorLmb/runEventTriggeredDistributedLmbFilter.m`, `collectCurrentFusionInputs` and configuration defaults | Missing neighbor mode defaults to `renormalize`; `self` already implemented. |
| E5 | `tectonic --keep-logs main.tex` from `papers/icassp2027` | PDF build and rendered pages checked locally; see build log. |
| E6 | `octave --no-gui --quiet --eval "addpath(genpath(pwd)); analyzeSetErrorBudgetV279();"`; `RUN/GA/dynamic_topology/evidence/tracking_aligned_v279/set_error_budget_seed1301/SET_ERROR_BUDGET_V279.md` | Exit 0. Sparse squared-OSPA count share: 99.567% M24; 98.074--99.337% X36. X36 common-cell RMSE: 19.329 / 31.394 m. |
| E7 | The exact V278 launch command is retained in `RUN/GA/dynamic_topology/V278_MISSING_PACKET_SELF_WEIGHT_DESIGN.md`; completed result is `RUN/GA/dynamic_topology/evidence/tracking_aligned_v278/x36_missing_packet_self_seed1301/MISSING_PACKET_SELF_WEIGHT_V278.md` and its local MAT. | Process/session 43145 exit 0: `V278 complete: E 131.961; RMSE 20.263; consistency 139.273; card 18.518; bytes 60316640; M24 follow-up=0`. |
| E8 | `octave --no-gui --quiet --eval "addpath(genpath(pwd)); analyzeObservationReachabilityV280();"`; `RUN/GA/dynamic_topology/evidence/tracking_aligned_v280/observation_reachability_seed1301/OBSERVATION_REACHABILITY_V280.md` | Exit 0; M24/X36 global blackout 2.773%/2.734%; sparse ideal coverage within eight steps 81.261%/63.183%. |
| E9 | `figures/exportPaperFigureData.m`, `figures/plot_paper_figures.py`, source CSVs and figure manifest | Existing seven episode rows exported without rounding; two Python vector figures generated without filtering. |
| E10 | Ramachandran et al., IEEE TCNS 8(2):609--620, 2021; DOI 10.1109/TCNS.2021.3059794; author preprint https://arxiv.org/abs/2004.07197 | Prior topology/weight reconfiguration and subsequent robot repositioning explicitly distinguished from fixed-motion sparse LMB routing. |
| E11 | `octave --no-gui --quiet --eval "addpath(genpath(pwd)); analyzeExistenceLossLocalizationV281();"`; `RUN/GA/dynamic_topology/evidence/tracking_aligned_v281/m24_existence_loss_seed1301/EXISTENCE_LOSS_LOCALIZATION_V281.md` | Exit 0; 72 receiver snapshots and 1,124 label records; 659 weighted log odds negative before spatial overlap; probability identity residual <=2.22e-16. |
| E12 | `RUN/GA/dynamic_topology/V282_EXISTENCE_STAGE_TRACE_DESIGN.md` retains the launch command; `evidence/tracking_aligned_v282/x36_prefix2_integration_seed1301/EXISTENCE_STAGE_TRACE_V282.md` under `RUN/GA/dynamic_topology/` retains the completed short check. | Corrected integration session 61818 exit 0; 72 receiver-time cells, 1,728 label stages; E-OSPA/finite-RMSE differences 0 and finite masks match. Source `fbf17cd`. The subsequent 40-step completion is E13. |
| E13 | `RUN/GA/dynamic_topology/evidence/tracking_aligned_v282/x36_prefix40_seed1301/EXISTENCE_STAGE_TRACE_V282.md`, `receiver_stages.csv`, `label_stages.csv`; local raw trace MAT. | Session 4122 exit 0; 1,440 receiver-time and 34,424 label stages; all E-OSPA/finite-RMSE values and masks match the stored reference prefix. Filter time 609.8 s, excluding scene generation and offline analysis. |
| E14 | `octave --no-gui --quiet --eval "addpath(genpath(pwd)); analyzeObservationLineageV283('RUN/GA/dynamic_topology/evidence/tracking_aligned_v282/x36_prefix40_seed1301/EXISTENCE_STAGE_TRACE_V282.mat');"`; `OBSERVATION_LINEAGE_V283.md` and `observation_lineage_by_time.csv` in that directory. | Original offline analysis session 28909 and report-expansion session 42724 exit 0. Late mean never-informed weight 0.0083 versus zero-current-pD weight 0.7949. All 40 opened steps used; no counterfactual intervention. |
| E15 | DOI content negotiation for G. Li et al., 10.1016/j.sigpro.2019.107246; S. Li et al., 10.23919/ICIF.2018.8455250; Wang et al., 10.1016/j.sigpro.2018.04.010. Sources and scope are recorded in `RUN/GA/dynamic_topology/V283_OBSERVATION_EVIDENCE_FINDING.md`. | CA-GCI author PDF concerns GM-PHD clustering/compensation. Official conference abstract establishes multi-view LMB prior art; Wang publisher abstract/introduction establishes centralized per-label information-based weights. No claim of full LMB algorithm replication. |

## Verification Record

Self-check only. Existing paired numeric artifacts and runtime weighting rules
were inspected; V277 was executed. The draft's mathematical arguments were
checked by the producing agent, not an independent reviewer. This is a draft,
not a validated final paper. User requested focus on method decisions rather
than additional adversarial or hash audits.

On 2026-09-05 the final draft built with Tectonic (exit 0) and all four PDF
pages were rendered and visually inspected. There are no overfull boxes;
Tectonic reports underfull spacing and a bibliography rerun-consistency warning.
The seven references render and resolve in the PDF. The local evidence lint
passed. The canonical Lark document was updated in place to revision 1198,
then its key-result paragraph, author information and best-method table were
read back. No independent verification or submission readiness is implied.

V279 reuses the saved metric arrays, the exact metric definitions and the
regenerated truth cardinalities. The two possible estimate counts are checked
against the lower cardinality floor and an upper bound from the recorded
RMSE matching; unresolved cases retain intervals (121/5760 for sparse X36).
This is a producing-agent self-check, not independent verification. No filter,
runtime source or V278 gate was changed. Its result is added to the progress
guide; the English manuscript now includes this readout and its count bounds.
Command `python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py papers/icassp2027/EVIDENCE.md`
returned `PASS: papers/icassp2027/EVIDENCE.md` after adding E6.

The canonical Lark method-design section and its later framework table were
updated in place through revision 1216. They now distinguish the implemented
V242 backbone, the unconfirmed V278 receiver rule, and the target-set bottleneck.
One SVG schematic was inserted after the section introduction (revision 1217;
block `doxlgarEJEouJwtTUYsrrC2TXkd`, board `Yv1nwom4Bh15o5batldjLRr6prc`).
Two wording refinements updated that same board, without inserting another one.
The primary agent visually checked both the final local PNG and the actual
exported Lark preview under `figures/`. The Lark export uses black text and
additional canvas whitespace, but the diagram remains complete and legible.
This is a document/layout check, not independent scientific validation.

V278 subsequently completed with session exit 0. Its saved source commit is
`eb7bc57f541af00f37636b61a1d2edaa919c51e6`; the runtime source was not edited
while it ran. The executed receiver ablation fails the frozen M24 follow-up
gate, so no new filter arm was launched. A same-finite-cell check also finds
RMSE deterioration (19.990875/18.953983 m, -5.470572%); the exact readout is
retained in E7. V280 completed without filtering and checked replayed message
counts against every saved reference arm. These are producing-agent checks.

The revised manuscript was built with `tectonic --keep-logs main.tex` (exit 0)
and rendered with `pdftoppm -r 105 -png main.pdf ../../tmp/pdfs/icassp2027-v280/final`.
All five pages were visually inspected: technical content ends on page four;
page five contains declarations and references only. No overfull boxes remain;
the known underfull-spacing and bibliography-rerun warnings persist. V278 and
V279 are included in the manuscript; V280 remains a diagnostic in the local
progress guide and experiment record. No independent validation is claimed.

Lark prose and the persistent best-method table were updated in place through
revision 1231. A local keyword readback confirmed the new V278 row and V280
interpretation, with no obsolete pending-V278 status among the queried terms.
The same evidence-lint command returned `PASS: papers/icassp2027/EVIDENCE.md`
after adding E7--E8. The next bounded diagnostic is specified in
`RUN/GA/dynamic_topology/V281_EXISTENCE_LOSS_LOCALIZATION_DESIGN.md`; it is a
plan, not an executed result or a new method claim.

The local method SVG and PNG now label V278 as completed and explicitly state
the X36 tradeoff against V242. The local PNG was rendered and visually checked.
The online board still contains the earlier pending-status wording: a raw-node
update was rejected by field validation, and a subsequent export confirmed the
original board remained intact. Browser editing also timed out. No SVG overwrite
was performed; that fallback awaits the user's confirmation because it rebuilds
the board's node semantics. The retained online preview is therefore the earlier
version, not evidence that the final status-text update reached Lark.

On 2026-09-06 the user explicitly approved that SVG replacement. The same
board `Yv1nwom4Bh15o5batldjLRr6prc` was updated successfully with idempotency
token `20260906-method-mainline-svg-sync-v1` (returned node `o1:103`). No new
document or whiteboard block was created. The fresh exported preview replaces
the earlier local JPG; the primary agent visually confirmed `V278 已测试`
and the explicit `相对原稀疏骨干` comparison. This resolves the pending
online-diagram status above; the SVG import rebuilt the board's internal
node structure as authorized.

The 2026-09-06 paper revision replaces the stacked pipeline with a Python
method schematic and adds a payload/E-OSPA tradeoff figure. The figure
contract was written before plotting; CSV exports come from the saved MAT
results, not the rounded table. The exact sparse/fixed gains agree with
E1--E2. Vector PDF, editable-text SVG and 600-dpi PNG exports are retained.
The primary agent inspected both figures and all five final manuscript
pages rendered with Python/PDFium. Technical content ends on page four;
page five contains declarations and eight references. Tectonic exits 0,
with no overfull boxes; underfull spacing and the existing bibliography
rerun-consistency warning remain. V280 is now a compact, explicitly idealized
diagnostic in the paper. E10's metadata was retrieved by DOI content
negotiation and its relevance checked against the author preprint.
This is producing-agent self-check only, not author approval or a finding
that the current novelty and generalization gaps are resolved.

V281 then executed only the saved V242 reference assignment at each of the
three registered M24 anchors. The observer uses cached post-local-update,
pre-topology/pre-fusion posteriors, scheduled senders in runtime order, the
paired directed uniforms, omitted empty/unavailable packets, and the actual
MAP-cardinality extractor. There were four lost incoming packets, none of
which removed a label's last scheduled input occurrence, and no delivered
empty input. This does not imply zero loss of useful evidence. The fusion,
MAP and reference-assignment sources have no diff from the cache generation
commit to the replay runtime commit. No filter or runtime source was changed.
Raw receiver/label CSVs and the analysis script are retained; the local MAT
also retains per-source existences and active weights. The 0.5/0.9 levels
describe input evidence only and are not candidate extraction thresholds.
This limited self-check narrows the next trace to earlier local updates,
propagation and current existence pooling; it does not establish a new
tracking improvement. Main-paper performance values remain unchanged.

The canonical Lark document was updated in place to revision 1232 with only
the decision-relevant V281 interpretation after the existing V280 paragraph.
A local keyword readback confirmed both new paragraphs, including the
restriction to three M24 anchors and the remaining X36 evidence gap. The
existing best-method table and whiteboard were not changed by this prose
update. The detailed 72-receiver trace remains in the experiment record.
The evidence lint returned `PASS: papers/icassp2027/EVIDENCE.md` after E11
was added. No new filter is running at this handoff.

At the prior V282 handoff, an unchanged X36 reference prefix was running
in session 4122, with runtime and capture source frozen at `fbf17cd`. Its
two-step integration check (E12) passed after fixing Octave MAT serialization;
the 40-step run had reported the start of step 6, not completion. The short
trace numerically confirms neutral local existence updates at its zero-pD
label stages (maximum absolute change 6.94e-18). It does not identify the
full-episode bottleneck or justify a policy change. Raw trace MAT files and
logs remain local; the compact short-run report is retained in git.

The manuscript now states the component-mean FoV detection approximation
and distinguishes MAP selection from posterior pruning. Duplicate prose was
shortened, and hyperlink borders hidden without changing the template style.
Tectonic completed with exit 0; the primary agent inspected all five final
rendered pages under `tmp/pdfs/icassp2027-v282/verified-*.png`. Technical
content ends on page four; page five contains declarations and references.
No overfull boxes remain; underfull spacing and the existing bibliography
rerun-consistency warning remain. Figures and performance values are
unchanged. Lark receives no routine integration-status paragraph or new
best-method row from that integration-only work.

V282's 40-step capture and analysis have now completed (E13). V283 reuses
the saved inputs and actual active weights to propagate observation-opportunity
flags (E14), with sent-label presence respecting the existing 0.01 payload
threshold. Its report includes both weak-pool denominators and counts lacking
a strong input; the algorithm was not changed by this reporting expansion.
No truth matching, policy intervention or new full episode was performed.
The finding narrows the design question to useful evidence strength and age
over repeated hops, while explicitly retaining cold-start and accumulated
spatial effects as unresolved possibilities. No filter is running.

The paper's multi-view paragraph was corrected, two verified LMB references
added, and the CA-GCI citation upgraded to its journal record. A compact V283
readout was added without changing any best-method performance value or figure
data. Tectonic completed with exit 0. All five final pages under
`tmp/pdfs/icassp2027-v283/page-*.png` were visually inspected: technical
content including the conclusion ends on page four; page five contains only
declarations and ten references. No overfull boxes remain. Underfull spacing
and the known bibliography-rerun consistency warning remain. This is a
producing-agent layout and evidence self-check, not submission readiness.

Canonical Lark was updated in place through revision 1243: the new X36
decision-relevant finding, related-work comparison, TL;DR and next-step
paragraphs were synchronized. Stale statements that the X36 comparison was
pending and that the V278 trial was next were corrected. Local keyword
readback confirmed the new finding/table and multi-view comparison. The
existing best-method table and approved whiteboard were preserved; no new
document or board was created. Routine integration status stayed out of Lark.
The existing evidence-lint command returned `PASS: papers/icassp2027/EVIDENCE.md`
after adding E13--E15; no additional adversarial or hash audit was run.

## Risk and Escalation

An overbroad interpretation would overstate tracking quality or total bandwidth
savings. The draft explicitly states both limitations. Author review, novelty
comparison and independent scene/seed results are needed before submission.

## Reproducibility

The M24 and X36 source commits are retained in their reports. The X36 stored
MAT files remain in the `v274-x36-minimum-backbone` worktree. V278 reuses that
V242 result path and runs only the new arm; its frozen command and stopping
criterion are in `RUN/GA/dynamic_topology/V278_MISSING_PACKET_SELF_WEIGHT_DESIGN.md`.
The official template archive is retained locally. Bib metadata was retrieved
with DOI content negotiation for 10.1109/TSP.2014.2323064,
10.48550/arXiv.1501.01579, 10.48550/arXiv.1903.06985 and
10.1109/TSP.2008.920469, 10.1016/j.automatica.2013.11.042,
10.1109/TIT.2006.874516 and 10.48550/arXiv.1902.09825, plus E15's three
multi-view journal/conference records, with title/claim
checks against the primary papers and the metric implementation.

## Open Issues

Useful observation-evidence preservation over repeated lossy hops, with
cold-start and cumulative spatial effects not causally isolated;
faithful multi-view LMB algorithmic baselines; weakest-formation behavior;
multi-seed and non-radial validation;
control-inclusive communication; routing-specific prior-art comparison;
extension of the exact fixed-density bound to approximate recursive filtering.

## Recommendation

Retain one mainline: sparse causal routing with delivery-aware receiver
behavior (C1--C4). The single-factor ablation is complete and fails the frozen
follow-up gate (C7). Keep its current numeric tradeoff in the best table,
without starting an M24 extension or scanning fallback parameters.
Prioritize successful target-set recovery and preservation (C5--C6), not
further position-only or matrix-mixing optimization. V280 separates global
visibility from time-respecting path opportunity but does not trace actual
posterior evidence (C8). V281 finds weak inputs already before the final
spatial term at three M24 anchors (C9). The completed X36 trace and lineage
analysis (C10--C11) distinguish historical opportunity access from current
evidence strength. Do not treat low late untouched-prior mass as ruling out
an early cold-start effect. Design one bounded evidence-preservation candidate
with explicit source, age, duplicate-information handling and metadata cost;
compare against relevant multi-view LMB prior art (C12), rather than claiming
novelty for information-based label weighting alone. No new candidate is frozen
or validated yet, and the joint M24/X36 success objective remains unmet.
