# Figure 2 Prompt

## Goal

Create a paper-ready method diagram for the adaptive fusion-weight factorization used in the GA-LMB paper.

## Prompt

Draw a clean academic block diagram of the adaptive fusion-weight computation pipeline. The diagram should begin with a neighborhood input set and an availability mask, then show a shared score backbone composed of covariance quality, realized link quality, and existence confidence. After the shared backbone, split the flow into two branch-specific paths: a spatial-weight path and an existence-weight path. The spatial path should emphasize covariance and link quality; the existence path should emphasize link quality and existence confidence. After the branch split, show a weak structure-aware branch prior that lightly modulates both paths, with the spatial-side prior visually stronger than the existence-side prior. Also show a compact FID-FIA cue that modulates only the existence-weight path; do not show the internal FID-FIA computation. After that, show EMA smoothing and minimum-weight safeguarding before the final normalized spatial and existence fusion weights. The visual hierarchy should make it obvious that the core method is the shared backbone, while branch decoupling and the two branch-specific refinements are secondary refinements. Use a compact, publication-ready style with simple rectangular blocks, directional arrows, and short labels only.

## Required Elements

- neighborhood inputs
- availability mask
- covariance-quality term
- realized link-quality term
- existence-confidence term
- shared backbone score
- spatial-weight path refinement
- existence-weight path refinement
- weak structure-aware branch prior
- FID-FIA cue as an existence-branch-only modulation
- EMA smoothing
- minimum-weight safeguard
- final normalized spatial and existence weights

## Style Constraints

- academic paper diagram style
- white background
- flat vector blocks and arrows
- minimal color coding
- clear hierarchy between shared backbone and branch-specific refinements
- no decorative icons or gradients

## Do Not

- do not turn it into a crowded flowchart
- do not include too much formula text inside the figure
- do not visually imply that structure-aware refinement is the dominant term
- do not show the internal FID-FIA derivation or calculation details
- do not use excessive colors
