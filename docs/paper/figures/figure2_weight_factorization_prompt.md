# Figure 2 Prompt

## Goal

Create a paper-ready method diagram for the adaptive fusion-weight factorization used in the GA-LMB paper.

## Prompt

Draw a clean academic block diagram of the adaptive fusion-weight computation pipeline. The diagram should begin with a neighborhood input set and an availability mask, then show a shared score backbone composed of covariance quality, realized link quality, and existence confidence. After the shared backbone, split the flow into two branch-specific refinements: a spatial branch and an existence branch. The spatial branch should emphasize covariance and link quality; the existence branch should emphasize link quality and existence confidence. After the branch split, show a weak structure-aware refinement stage that lightly modulates both branches, with the spatial side visually stronger than the existence side. After that, show EMA smoothing and minimum-weight safeguarding before the final normalized spatial and existence fusion weights. The visual hierarchy should make it obvious that the core method is the shared backbone, while branch decoupling and structure-aware refinement are secondary refinements. Use a compact, publication-ready style with simple rectangular blocks, directional arrows, and short labels only.

## Required Elements

- neighborhood inputs
- availability mask
- covariance-quality term
- realized link-quality term
- existence-confidence term
- shared backbone score
- spatial branch refinement
- existence branch refinement
- weak structure-aware refinement
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
- do not use excessive colors
