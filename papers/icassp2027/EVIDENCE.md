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
10.1109/TIT.2006.874516 and 10.48550/arXiv.1902.09825, with title/claim
checks against the primary papers and the metric implementation.

## Open Issues

Earlier local-update/recursive transport existence loss, with an X36 trace
still missing; weakest-formation behavior;
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
spatial term at three M24 anchors (C9). Trace earlier local updates and
recursive propagation before changing an eta floor or output threshold;
retain the current input-weighting effect as a distinct possible mechanism.
