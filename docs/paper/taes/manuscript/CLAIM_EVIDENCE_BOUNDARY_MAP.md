# TAES Claim-Evidence-Boundary Map

This file is a pre-submission audit artifact for the TAES manuscript package. It is not a separate data source and should not be cited as evidence. Its purpose is to keep every paper-facing claim tied to a manuscript location, a report-driven artifact, and an explicit boundary before the final portal upload or reviewer response.

## Scope

- Manuscript: `main.tex`
- Target venue: IEEE Transactions on Aerospace and Electronic Systems (TAES)
- Paper type: Regular Paper, technical area `Target Tracking and Multi-Sensor Systems`
- Core claim: after target-wise arithmetic-average (AA) weights reach the existence and spatial consumers, residual labeled multi-Bernoulli (LMB) fusion errors can remain because component correspondence is missing; a graph-local reference, assignment, and first-two-moment barycenter projection repairs this output-level correspondence for active tracks under the tested scenarios.
- Evidence rule: only claims supported by manuscript equations, tracked validation reports, generated fragments, verifier outputs, or fixed-design boundary checks may be used as paper-facing claims.
- Boundary rule: future maneuver/crossing, covariance/reliability, and recursive-online protocols are planning artifacts, not current manuscript evidence.

## One-Sentence Argument

In distributed LMB tracking under unreliable peer-to-peer communication, we show that residual errors can persist after target-wise AA weight routing because local Bernoulli components lack a shared correspondence contract; a graph-local reference, assignment, and first-two-moment barycenter projection repairs active output-track label/moment correspondence, supported by paired N50 ablation, held-out replication, and fixed-parameter boundary checks, with recursive lifecycle, covariance-consistency, and maneuver/crossing behavior outside the current claim.

## Reader Path

The manuscript should answer reviewer questions in this order:

1. Relevance: unreliable communication makes local labeled tracks difficult to compare across sensors.
2. Novelty: the paper treats label sets as a correspondence contract rather than as bookkeeping attached to scalar AA/KLA weights.
3. Trust: structural propositions and fixed-design ablations separate label copying from matched posterior barycentering.
4. Reuse: the method is a graph-local active-output projection with explicit inputs, costs, and source-code/evidence artifacts.
5. Meaning: the contribution is bounded to output-level correspondence repair and does not claim recursive LMB correctness, covariance consistency, or general label-management optimality.

## Section Job Map

| Manuscript section | Primary job | Must preserve | Must avoid |
| --- | --- | --- | --- |
| Abstract | State the residual correspondence failure, the graph-local projection, and the main full-versus-reference evidence. | Active-output scope, N50 paired evidence, held-out separation, fixed-parameter boundary checks. | Presenting the method as a new scalar-weight optimizer or recursive LMB update. |
| Introduction | Move from networked tracking relevance to the unresolved component-correspondence gap after scalar weight routing. | The `correspondence contract` framing and the distinction between probability-mass allocation and component matching. | A broad claim that AA/KLA fusion is generally inadequate. |
| Related Work | Position AA/KLA, heterogeneous RFS fusion, assignment, and consensus as adjacent foundations. | The complementarity claim: correspondence projection is orthogonal to density-pooling weights. | A paper-by-paper catalog without the missing-correspondence throughline. |
| Problem Formulation | Define the local LMB outputs, graph neighborhood, and missing correspondence map. | The active-track output-projection boundary. | Replacing the upstream Bernoulli existence consumer. |
| Method | Explain reference selection, assignment, moment barycenter, graph locality, and cost. | Reproducible mechanism detail and existence pass-through wording. | Hiding the assignment cost or implying global label-dictionary access. |
| Structural Properties | Give conditional algebraic support and formal boundaries. | Fixed-time active-output analysis scope, sufficient assignment stability, first-two-moment least-squares projection. | Claiming finite-round optimality or guaranteed assignment under crossings/births/deaths. |
| Experimental Setup | Define the mechanism-isolation protocol and fixed parameters. | Same seeds/measurements across arms, tuned scalar-weight baseline, reference-only ablation, no per-scenario search. | Treating held-out/boundary runs as tuning sources. |
| Results | Walk the reader through primary gains, GA context, ablation separation, and held-out replication. | Table/figure order: primary N50, contextual GA, paired ablation, held-out check. | Overstating contextual GA rows as paired significance evidence. |
| Discussion and Limitations | Interpret why the ablation supports matched barycentering and where the method can fail. | Assignment ambiguity, covariance/reliability limits, recursive lifecycle boundary, formation-family scope. | Turning limitations into generic disclaimers disconnected from the method. |
| Conclusion | Restate the bounded contribution and required recursive safeguards. | Output-level correspondence module language. | Promising validated recursive LMB deployment. |

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
| Matched posterior barycenters, not label copying alone, drive the main spatial tracking gain. | Results, `tab:paired`, `fig:n50`, Discussion | Main N50 full-vs-reference-only ablation; held-out N50 replication; harsh-loss, topology-ring, and partial-FOV generated evidence | Report-driven fragments plus independent verifier for the main N50 local metrics | Do not collapse reference-only and full effects; cardinality effects are shared by both arms. |
| The full operator improves both network agreement and local tracking metrics relative to the tuned spatial-KLA AA baseline in the primary N50 validation. | Abstract, Results, `tab:n50`, `tab:paired`, `fig:n50` | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED1_20260623_232622.md`; `generated/n50_evidence.json`; `generated/N50_VERIFICATION_REPORT.md` | Primary report parsed and independently recomputed for network, runtime, and local metrics | State the scenario: eight-sensor tiered packet-loss formation with fixed method parameters. |
| Held-out base-seed N50 replication preserves the full-versus-reference separation. | Abstract, Results, `tab:heldout` | `RUN/AA/AA_BALANCED_CARDINALITY_VALIDATION_N50_SEED11_20260624_094913.md`; `generated/HELDOUT_N50_MANIFEST.md`; `generated/heldout_n50_section.tex` | Parsed N50 held-out gate with paired CI, wins, and sign-test values | Use as robustness evidence, not as a source for selecting parameters. |
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
