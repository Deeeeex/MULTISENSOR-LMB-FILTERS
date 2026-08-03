# V40 conditional-preserving reweight diagnostic

## Decision

**Stop V40 before continuation or tracking.**  Exact conditional-preserving
reweighting removes a real counterfactual confound, but it does not repair the
misalignment between current one-round homogenization and the temporal
label-protection mechanism.  The preregistered four-cache mechanism gate fails
on two of its four requirements.

No runtime selector was executed, no state estimate was scored, and no target
truth or future outcome entered the controller.  These values remain
development diagnostics rather than a formal truth-free preflight because the
temporary cache helper reconstructs a complete deterministic scenario before
building the sanitized current context.

- Source protocol SHA-256: `02a6bc2951621eb22a56d8763d28bd922bcc0fbb4323b958856d1da05f78c310`
- Source worktree commit: `99d0eefce7fc6d17053c34e9de277f421e738393`
- V40 base commit: `814e285`

| Seed | Cache file SHA-256 | Cache payload SHA-256 |
|--:|:--|:--|
| 41 | `6cbce9927a38a7338177ad72869cfddea5ba94e15393756678ed6af8088cc568` | `57ab8e04cbce481aad76738789a590b55a57f90e85487e8e9f1f0246a0131f6f` |
| 43 | `f731ee498ffbaafb50c5b7880634749933f5a95b047f499f6af48e1ed3a39552` | `bf97c2defe299651096b26c12a89107c04f6ffa1430eb99d7c0f534305eabcdb` |
| 47 | `bf7d93a489e279bbd85784cd8c6765ea0c0a37eead0356efbe20c4f418b7a908` | `9f3f82fe183db7f7368fa3f8b533cbb62105b166a70922f1c032f3063d4e9a83` |
| 53 | `6db59a5a9236d747af559d734a3c8aa3111224c5d10b08d13f392407e35cb5c6` | `c52f9539ed909b5e8525048ea9a83b5af003457e0bbc5c3857f81760a448c69a` |

## Gate result

| Frozen requirement | Result | Decision |
|:--|:--|:--:|
| Exact missing-input weight equivalence for every changed receiver | passed on all 16 singles | PASS |
| All five legacy requested-safe singles have lower one-round risk under V40 | 2 / 5 | **FAIL** |
| At least four of five remain safe with protection at least 2% | 5 / 5 | PASS |
| At least two of four cases contain a safe, positive-protection, non-positive-risk single | 1 / 4 | **FAIL** |

Across all 16 single-formation comparisons, V40 lowers one-round risk in only
6 cases.  The median V40-minus-v38 risk shift is approximately `+0.0302`
percentage points, so the clean counterfactual is slightly worse rather than
systematically better on this sample.  The number of safe singles increases
from 8 to 9 and the number passing the 2% protection threshold increases from
5 to 6, but the safe strict-feasible count remains 1.

## Per-action diagnostic

`Protection` and `Delta R` are reference-relative percentages.  Every action
saves two of 48 directed message opportunities.  `C` is V40 conditional
renormalization; `L` is the frozen v38 self-return action.

| Seed | Formation | C safe | C requested | C protection | C Delta R | L safe | L requested | L protection | L Delta R | C minus L risk |
|--:|--:|:--:|:--:|--:|--:|:--:|:--:|--:|--:|--:|
| 41 | 1 | yes | yes | +5.3687% | +1.1614% | no | no | +5.1969% | +1.0824% | +0.0790 pp |
| 41 | 2 | no | no | +3.6053% | +1.7288% | no | no | +3.3677% | +1.6719% | +0.0569 pp |
| 41 | 3 | yes | yes | +6.9726% | +3.8092% | yes | yes | +7.0206% | +3.7752% | +0.0340 pp |
| 41 | 4 | yes | no | +1.8774% | +0.2202% | yes | no | +1.9461% | +0.2504% | -0.0302 pp |
| 43 | 1 | no | no | -0.1524% | +0.1155% | no | no | -0.1713% | +0.0857% | +0.0298 pp |
| 43 | 2 | no | no | +0.5240% | -0.1100% | no | no | +0.5229% | -0.0943% | -0.0157 pp |
| 43 | 3 | yes | yes | +6.5205% | +1.6955% | yes | yes | +6.6699% | +1.7337% | -0.0382 pp |
| 43 | 4 | yes | yes | +6.1642% | +3.1094% | yes | yes | +6.0900% | +3.0305% | +0.0789 pp |
| 47 | 1 | yes | no | +1.2779% | -0.0933% | yes | no | +1.2581% | -0.0582% | -0.0351 pp |
| 47 | 2 | no | no | +2.7687% | +0.5235% | no | no | +2.7290% | +0.4790% | +0.0446 pp |
| 47 | 3 | yes | yes | +6.8470% | +3.3187% | yes | yes | +6.9273% | +3.3664% | -0.0477 pp |
| 47 | 4 | yes | yes | +6.4379% | +2.6962% | yes | yes | +6.3393% | +2.6445% | +0.0517 pp |
| 53 | 1 | no | no | +0.9734% | +0.7658% | no | no | +0.9959% | +0.7671% | -0.0013 pp |
| 53 | 2 | yes | no | +1.2746% | +0.3237% | yes | no | +1.2390% | +0.2333% | +0.0904 pp |
| 53 | 3 | no | no | +1.5017% | +1.0333% | no | no | +1.6635% | +1.0027% | +0.0307 pp |
| 53 | 4 | no | no | +0.1036% | +0.2431% | no | no | +0.1295% | +0.2268% | +0.0163 pp |

## Interpretation

V40 proves that the old self-return intervention was not the only reason for
the strict-risk failure.  Conditional preservation changes both risk and
retention only modestly and with mixed signs.  High-protection candidates
still usually increase current pairwise network agreement risk because they
intentionally prevent a conflicting cross-formation posterior from
homogenizing the receiver immediately.

The appropriate role split is therefore:

- entry into a sparse state: protection/utility plus structural and
  window-level constraints;
- recovery from a sparse state: one-round disagreement may order which mixing
  inputs should return first;
- paper-level safety: a scale-aware window contraction certificate and a
  proper-metric local posterior perturbation bound;
- tracking benefit: fresh paired multi-step experiments, not an internal
  one-round score.

V40 remains an isolated mechanism branch.  It does not change the frozen v38
source run and must not be promoted by relaxing this failed gate.

## Evidence boundary

The four files are frozen source caches for radial M24 seeds 41, 43, 47 and
53 at time 55.  The probe reads their posterior/history state and uses the
same current link page in both interventions.  The sanitized policy context
contains no truth or future outcome, but the diagnostic helper itself calls
the deterministic source-input builder.  A later formal preflight must instead
load a current observable-page manifest whose construction never materializes
truth, measurements, link uniforms, future drop pages or trajectory history.
