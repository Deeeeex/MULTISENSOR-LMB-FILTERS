# V226 bounded transfer core self-check

## Question

Can the label-wise KLA routing path support a deterministic, receiver-specific partial transfer that preserves self evidence and total label-wise weight, rejects both excessive existence loss and excessive existence gain, and falls back to ordinary fusion without reducing the charged payload?

The decision supported by this package is whether the V226 core is ready to be exercised on newly generated, same-version trajectory caches. It does not decide whether V226 improves tracking.

## Scope

Included: the two-sided Bernoulli log-odds projector, bounded dominant nonself weight transfer, full-grid deterministic selection, runtime fallback and diagnostics, V226 protocol/action-bank/runner plumbing, and focused synthetic semantics tests on branch `codex/v226-bounded-transfer` based on `19a2484`.

Excluded: old V223-V225 tracking outcomes, corrected X36/M24 trajectory results, online source/label selection, cross-seed generalization, and paper-level performance claims.

## Risk Tier

L2. This is code-backed experimental infrastructure that will determine later benchmark results; it may be implemented and tested, but paper-facing claims require independent verification and new paired experiments.

## Claims

| ID | Claim | Confidence | Evidence IDs | Caveats |
| --- | --- | --- | --- | --- |
| C1 | The projector can enforce a finite upper and lower ordinary-reference log-odds envelope while preserving unbounded-upper V223 behavior when the new option is omitted. | High | E1, E3 | Synthetic scalar cases only. |
| C2 | A bounded transfer preserves total label-wise KLA weight and the receiver's self weight, and fraction 1 matches the legacy full dominant-edge transfer. | High | E2, E3 | Synthetic identical-spatial-density case only. |
| C3 | V226 evaluates a frozen fraction grid per receiver and selects the numerically largest allowed fraction without a monotonicity assumption; no allowed fraction maps to ordinary-fusion fallback. | High | E1, E3, E8 | Synthetic runtime only; corrected trajectory outcomes remain pending. |
| C4 | Projection cannot create artificial byte savings: the protocol declares that transfer fraction does not reduce payload charge, and the runtime charge is applied before receiver projection. | High | E4, E8 | Actual attempted/delivered bytes remain to be measured on the corrected run. |

## Evidence Ledger

| ID | Type | Source or artifact | What it supports | Strength |
| --- | --- | --- | --- | --- |
| E1 | code | `common/projectLabelKlaExistenceRetentionByEtaV223.m`; `common/selectMaximumAllowedTransferFractionV226.m` | C1, C3 | strong |
| E2 | code | `multisensorLmb/fuseLmbPosteriorsByLabel.m` bounded transfer override | C2 | strong |
| E3 | command | `octave --no-gui --quiet --eval "addpath(genpath(pwd)); test_bounded_dominant_edge_transfer_v226;"` -> `test_bounded_dominant_edge_transfer_v226 passed` | C1, C2, C3 | strong |
| E4 | code | `common/getBoundedDominantEdgeTransferV226Protocol.m`; `multisensorLmb/prepareBudgetRecycledLabelInputRouteV221.m`; `multisensorLmb/runEventTriggeredDistributedLmbFilter.m` | C4 | medium |
| E5 | command | `octave --no-gui --quiet --eval "addpath(genpath(pwd)); which runEventTriggeredDistributedLmbFilter; which runFormationModeOpenedReturnScreen; which runSinglePassSemanticInputRoutingH3V221; test_partial_label_fusion_semantics_v90; test_canonical_lmb_gm_representation;"` -> all three functions resolved; partial-label test passed; canonical LMB-GM tests passed | C2, C3, C4 | medium |
| E6 | command | `git diff --check` -> exit 0 with no output | patch integrity | medium |
| E7 | command | `octave --no-gui --quiet --eval "addpath(genpath(pwd)); test_formation_b4_v45_filter_runtime_semantics;"` -> `PASS: FormationB4V45 filter runtime semantics tests` | main-filter regression coverage outside the inactive V226 branch | medium |
| E8 | command | `octave --no-gui --quiet --eval "addpath(genpath(pwd)); test_bounded_transfer_runtime_v226;"` -> `test_bounded_transfer_runtime_v226 passed` | C3, C4; both target receivers evaluated all 12 fractions, selected fraction 1, and retained the full 560 B route charge | strong |

## Verification Record

Independence status: `self-check only`. The producing agent ran deterministic focused tests, a one-page active V226 runtime integration with two receiver-specific decisions, legacy partial-label semantics, canonical GM representation checks, a pre-existing end-to-end main-filter runtime regression, and Octave parse resolution for the three modified runtime entry points. No independent verifier was used, so this package remains a draft engineering checkpoint and cannot certify tracking benefit or publication claims.

The falsification cases include an overconfident candidate rejected only by the new upper bound, an underconfident candidate rejected by the lower bound, a non-monotone allowed-mask selection, a no-safe-fraction fallback, self-weight preservation, total-weight preservation, and equivalence between bounded fraction 1 and the legacy full transfer.

## Risk and Escalation

If the implementation is wrong, later experiments could attribute gains to an unsafe or mischarged action. Before any paper-facing result, inspect receiver-level selected fractions and byte ledgers on new corrected caches, run same-version static/candidate pairs for M24 and X36, and obtain independent or human verification of the evidence package.

## Reproducibility

Run from `/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v226-bounded-transfer` with Octave 11.1.0:

```bash
git diff --check
octave --no-gui --quiet --eval "addpath(genpath(pwd)); test_bounded_dominant_edge_transfer_v226;"
octave --no-gui --quiet --eval "addpath(genpath(pwd)); which runEventTriggeredDistributedLmbFilter; which runFormationModeOpenedReturnScreen; which runSinglePassSemanticInputRoutingH3V221; test_partial_label_fusion_semantics_v90; test_canonical_lmb_gm_representation;"
octave --no-gui --quiet --eval "addpath(genpath(pwd)); test_formation_b4_v45_filter_runtime_semantics;"
octave --no-gui --quiet --eval "addpath(genpath(pwd)); test_bounded_transfer_runtime_v226;"
python3 /Users/dex/.codex/skills/auto-research/scripts/evidence_lint.py RUN/GA/dynamic_topology/evidence/tracking_aligned_v226/V226_BOUNDED_TRANSFER_CORE_SELF_CHECK.md
```

The frozen fraction grid is `[1, 0.75, 0.50, 0.40, 0.30, 0.25, 0.20, 0.15, 0.10, 0.075, 0.05, 0.025]`; both reference log-odds tolerances are `0.25`.

## Open Issues

- The corrected X36 seed-1301 trajectory collection is still running in the separate `v218-stratified-sampling` worktree.
- The V226 runtime branch has passed a synthetic active-route integration but has not yet been exercised on corrected cached posterior states.
- The source and label are still offline teacher choices; V226 is a mechanism screen, not an online policy.
- A partial transfer changes estimation influence but not payload size; communication gains must come from canonicalized payloads and withheld ordinary messages.

## Recommendation

Proceed to corrected-cache runtime execution only after the same-version X36 baseline finishes. Treat C1-C4 as a self-checked implementation checkpoint, not evidence that V226 improves E-OSPA, RMSE, consistency, or communication.
