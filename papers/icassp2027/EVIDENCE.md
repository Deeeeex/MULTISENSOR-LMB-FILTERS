# Research handoff: sparse causal LMB routing

## Question

Which single research direction is supported by the completed M24/X36 results,
and what can be written now in an independent ICASSP 2027 draft?

## Scope

V248 M24 and V274 X36 full episodes; V277 packet-only replay; implemented
V240/V242 routing; V278 single-factor receiver ablation; official ICASSP kit.
Old full/light equivalence and earlier fusion-code results are excluded.

## Risk Tier

L2: reproducible internal research draft. No submission or publication action.

## Claims

| ID | Claim | Evidence | Caveat |
|:--|:--|:--|:--|
| C1 | Sparse repair reduces attempted posterior bytes in both executed cases. | E1, E2 | One seed per scale; control traffic excluded. |
| C2 | X36 RMSE gain coexists with cardinality and consistency deterioration. | E2 | Conditional RMSE is not complete-set accuracy. |
| C3 | Planned balance and strong connectivity do not survive every packet realization. | E3, E4 | Packet-level replay, not label-wise state propagation. |
| C4 | Self fallback preserves surviving-neighbor weights but has no proven tracking gain. | E4 | V278 is running; no outcome is inserted into the paper. |

## Evidence Ledger

| ID | Artifact / command | Representative evidence |
|:--|:--|:--|
| E1 | `RUN/GA/dynamic_topology/evidence/tracking_aligned_v248/m24_temporal_task_coupled_formation_braid_seed1301/TEMPORAL_TASK_COUPLED_FORMATION_BRAID_V248_FULL_EPISODE.md` | Sparse: 122.462 E-OSPA, 12.183 RMSE, 36,675,624 B. |
| E2 | `RUN/GA/dynamic_topology/evidence/tracking_aligned_v274/x36_minimum_backbone_seed1301/` | Sparse: 132.192 E-OSPA, 19.329 RMSE, 60,090,416 B; cardinality error 18.597396 vs 18.455729. |
| E3 | `octave --no-gui --quiet --eval "addpath(genpath(pwd)); analyzeRealizedKlaBalanceV277();"` | Sparse packet-strong steps: 41/160 M24, 18/160 X36; non-double steps: 122/160, 143/160. |
| E4 | `multisensorLmb/runEventTriggeredDistributedLmbFilter.m`, `collectCurrentFusionInputs` and configuration defaults | Missing neighbor mode defaults to `renormalize`; `self` already implemented. |
| E5 | `tectonic --keep-logs main.tex` from `papers/icassp2027` | PDF build and rendered pages checked locally; see build log. |

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

V278 outcome; weakest-formation behavior; multi-seed and non-radial validation;
control-inclusive communication; routing-specific prior-art comparison;
extension of the exact fixed-density bound to approximate recursive filtering.

## Recommendation

Retain one mainline: sparse causal routing with delivery-aware receiver
behavior (C1--C4). Complete the single-factor ablation before another routing
heuristic. Keep the current numeric best table even when the joint gate fails.
