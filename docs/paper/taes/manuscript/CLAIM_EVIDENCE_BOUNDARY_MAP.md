# TAES Claim-Evidence-Boundary Map

This file is a pre-submission audit artifact for the TAES manuscript package. It is not a separate data source and should not be cited as evidence. Its purpose is to keep every paper-facing claim tied to a manuscript location, a report-driven artifact, and an explicit boundary before the final portal upload or reviewer response.

## Scope

- Manuscript: `main.tex`
- Target venue: IEEE Transactions on Aerospace and Electronic Systems (TAES)
- Paper type: Regular Paper, technical area `Target Tracking and Multi-Sensor Systems`
- Core claim: after target-wise arithmetic-average (AA) weights reach the existence and spatial consumers, residual labeled multi-Bernoulli (LMB) fusion errors can remain because component correspondence is missing; a graph-local reference, assignment, and first-two-moment barycenter projection repairs this output-level correspondence for active tracks under the tested scenarios.
- Evidence rule: only claims supported by manuscript equations, tracked validation reports, generated fragments, verifier outputs, or fixed-design boundary checks may be used as paper-facing claims.
- Boundary rule: future maneuver/crossing, covariance/reliability, and recursive-online protocols are planning artifacts, not current manuscript evidence.

## Terminology Ledger

| Canonical term | First-use definition in paper | Variants to avoid | Decision |
| --- | --- | --- | --- |
| LMB | labeled multi-Bernoulli (LMB) | labeled multi-Bernoulli filter when the abbreviation is already defined | Spell out once, then use LMB. |
| AA | arithmetic-average (AA) | arithmetic average without hyphen when used adjectivally | Use AA after first definition. |
| KLA | Kullback--Leibler average (KLA) | KL average | Use KLA after first definition. |
| OSPA | optimal sub-pattern assignment (OSPA) | ospa, OSPA distance without definition | Use OSPA after first definition. |
| E-OSPA | expected OSPA (E-OSPA) | expected optimal sub-pattern assignment after abbreviation is defined | Use E-OSPA in tables and results. |
| RMSE | root-mean-square error (RMSE) | RMS error | Use RMSE in tables and results. |
| FOV | field of view (FOV) | field-of-view after abbreviation is defined | Use FOV after first definition. |
| Neighborhood label-barycenter | graph-local reference, assignment, and matched moment-barycenter projection | label consensus, label barycenter, consensus projection when referring to the full operator | Use `neighborhood label-barycenter` for the full method. |
| Reference-only ablation | same reference selection and label canonicalization, without matched posterior barycenters | label copying only, medoid copy only | Use `reference-only` to identify the mechanism-control arm. |

## Claim-Evidence-Boundary Matrix

| Paper-facing claim | Manuscript location | Evidence / artifact | Verification level | Boundary / required wording |
| --- | --- | --- | --- | --- |
| Scalar AA/KLA weights allocate probability mass or trust but do not infer Bernoulli component correspondence when local labels disagree. | Introduction, Problem Formulation, Structural Properties | AA/KLA/RFS citations in `references.bib`; `Weighting is not matching` counterexample in `main.tex` | Conceptual, citation-backed, and formalized by a minimal two-target label-swap example | Do not claim that AA/KLA methods are generally inadequate; the claim is about residual component correspondence after weights are routed. |
| The proposed operator canonicalizes labels by a graph-local reference and assignment before replacing matched Gaussian states by first-two-moment barycenters. | Method, Fig. 1, implementation outline | `generated/method_pipeline.tex`; `figures/fig_method_pipeline.svg`; algorithm box in `main.tex`; `scripts/render_figures.py` | Source-backed, machine-generated, and rendered in the PDF | Keep the method as an output-space projection layer; do not call it a new density-pooling rule. |
| The projection rewrites active output labels and Gaussian moments but does not re-estimate Bernoulli existence probabilities. | Problem Formulation, Matched Moment Barycenter, Structural Properties, Experimental Setup | `main.tex`; implementation wording aligned with `multisensorLmb/applyCrossLocalLabelConsensusProjection.m` and `multisensorLmb/runDistributedLmbFilter.m` | Code-aligned and readiness-checked by source markers | Tie existence claims to the upstream AA existence consumer and convexity proposition. |
| The neighborhood implementation is graph-local and does not require a global label dictionary. | Graph Locality and Computational Cost, Fig. 1, implementation outline | `main.tex`; `RUN/AA/runAaBalancedCardinalityValidation.m`; generated method figure | Code path and manuscript text agree | Do not present centralized or full-topology runs as the main deployed setting. |
| Under fixed correct correspondence, the moment update is the least-squares representative in first-two-moment coordinates and equals the first two moments of an equally weighted matched Gaussian mixture. | Structural Properties | `Moment-space projection` proposition and proof in `main.tex` | Algebraic identity with positive-semidefinite covariance expression | This is not a covariance-consistency or reliability-optimality guarantee. |
| Under stable matching and connected repeated averaging, matched moment coordinates converge to the centralized equal-weight moment barycenter. | Structural Properties | Analysis-scope assumption; stable-matching consensus-limit proposition in `main.tex` | Theorem-level conditional claim | Keep finite-round results and recursive deployment separate from this limit statement. |
| Matched posterior barycenters, not label copying alone, drive the main spatial tracking gain. | Results, Fig. 3, Discussion | Main N50 full-vs-reference-only ablation; held-out N50 replication; harsh-loss, topology-ring, and partial-FOV generated evidence | Report-driven fragments plus independent verifier for the main N50 local metrics | Do not collapse reference-only and full effects; cardinality effects are shared by both arms. |
| The full operator improves both network agreement and local tracking metrics relative to the tuned spatial-KLA AA baseline in the primary N50 validation. | Abstract, Results, Tables I and III, Fig. 3 | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md`; `generated/n50_evidence.json`; `generated/N50_VERIFICATION_REPORT.md` | Primary report parsed and independently recomputed for network, runtime, and local metrics | State the scenario: eight-sensor tiered packet-loss formation with fixed method parameters. |
| Held-out base-seed N50 replication preserves the full-versus-reference separation. | Abstract, Results, held-out table | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED11_20260624_094913.md`; `generated/HELDOUT_N50_MANIFEST.md`; `generated/heldout_n50_section.tex` | Parsed N50 held-out gate with paired CI, wins, and sign-test values | Use as robustness evidence, not as a source for selecting parameters. |
| Harsh packet loss, topology-ring, and partial-FOV checks broaden the boundary evidence without forming a tuning loop. | Discussion, Supplementary/response evidence package | `generated/STRESS_HARSH_MANIFEST.md`; `generated/SCENARIO_FAMILY_MANIFEST.md`; `generated/stress_harsh_summary_sentence.tex`; `generated/scenario_family_summary_sentence.tex` | Fixed-parameter generated fragments and source hashes | Keep these as boundary checks within the formation family; they do not replace maneuver, covariance, or recursive-online validation. |
| Contextual GA rows show useful reference performance but are not paired AA-vs-GA significance-test evidence. | Results, contextual reference table | `RUN/GA/GA_TIERED_LINK_ABLATION_N50_SEED1_20260621_183039.md`; `generated/REFERENCE_BASELINE_MANIFEST.md` | Report-driven contextual comparison | Do not claim statistical superiority over GA from this table alone. |
| Runtime overhead is a relative paired prototype cost dominated by assignment and output rewriting. | Graph Locality and Computational Cost, Results runtime paragraph | `RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_20260623_232621.log`; `generated/N50_VERIFICATION_REPORT.md` | Runtime recomputed from trial logs | Do not present absolute Octave seconds as hardware-independent implementation benchmarks. |
| The current paper is content-ready under metadata allowance but not portal-submission-ready until final author/funding/repository/cover-letter fields are replaced. | README, Submission Package Index, generated readiness report | `generated/SUBMISSION_READINESS_REPORT.md`; `generated/submission_readiness.json`; `COVER_LETTER_AND_METADATA_DRAFT.md` | Machine-checkable readiness status | Metadata placeholders remain blocking for actual portal submission. |

## Explicit Non-Claims

- The method does not replace AA/KLA density fusion.
- The method does not optimize scalar fusion weights.
- The method does not guarantee correct assignment during close crossings, births, deaths, or cardinality disagreement.
- The equal moment barycenter is not a covariance-consistency guarantee.
- The current paper does not validate recursive online LMB feedback with lifecycle guards.
- The full-topology N50 run must not be used in manuscript claims until a complete report is parsed through `evidence_sources.json` and the scenario-family extractor.

## Final Paper-Readiness Use

Before final submission, use this map as a checklist:

1. Every strong claim in `main.tex` should appear here or be downgraded.
2. Every evidence row should point to a tracked report, generated artifact, verifier, manuscript equation, or citation.
3. Every robustness statement should state its scenario boundary.
4. Any new result should first enter `evidence_sources.json`, generated manifests, and readiness checks before entering the abstract, results, cover letter, or response text.
