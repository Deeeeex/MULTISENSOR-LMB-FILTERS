# V84: braided-handover information-flow opportunity

## Why this scene

The merge-split experiments established a useful negative result: a legal
alternative cross-formation sender can be available without carrying enough
task-relevant information to improve tracking. Increasing its KLA weight then
cannot create headroom. The next scene must therefore contain a real reason for
the communication graph to change.

The braided-handover geometry supplies that reason. Targets move through a
sparse chain of formations. For roughly half of the target-time pairs only one
formation has direct FoV support, and every target group changes its directly
observing formation at least once. M24 and X36 retain nearly equal local sensor
load and visible-formation count, while X36 adds propagation distance rather
than denser local sensing.

## First-principles hypothesis

Label-wise KLA geometrically combines existence evidence. During a handover,
two opposite errors are possible:

1. a receiver-supported track is multiplied by a low-existence posterior from
   a formation that currently has no observation support, suppressing a useful
   label;
2. a newly observing formation is not selected as the cross-formation sender,
   so its new label support does not reach the adjacent receiver.

Dynamic routing is useful only when it avoids the first error or enables the
second under the same communication budget. V84 therefore looks for physical
sender--receiver substitutions whose sender has current label support absent
from both the receiver and incumbent sender, while rejecting substitutions
that reduce receiver- or incumbent-supported labels. Candidate effects are
computed by the repository's bounded componentwise powered-GM KLA path, which
retains multiple spatial modes. This is a numerical mixture-aware
approximation, not an exact closed-form power of an arbitrary Gaussian
mixture. The association signal explains where useful information originates;
it does not replace the KLA counterfactual. Neither signal reads truth or
future tracking loss.

The two defining eligibility conditions are applied before the expensive KLA
counterfactual: an alternative must come from a different formation than the
incumbent sender, and must have at least one label with stronger current
association support than both the receiver and incumbent. This ordering only
removes candidates that cannot satisfy V84 by definition; it does not alter
the score or thresholds of any eligible candidate.

## Frozen development scan

- scenes: `m24-formation-fov-braided-handover` and
  `x36-formation-fov-braided-handover`;
- development seed: `41`;
- current-state grid: `t = 40:4:140`;
- reference graph: current physical formation path with the registered
  fixed-budget residual tour;
- permitted inputs: current local LMB posteriors, current association support,
  current physical links and reliability, and two past selected topologies;
- fusion reference: receiver-first, heavy-message, componentwise powered-GM
  mixture-aware KLA with the same bounded component settings used by the
  tracking reference;
- closed inputs: truth, future measurements, future links, and tracking
  outcomes.

The cache run is only a source trajectory. After it finishes, the opportunity
scan will rank times and directed substitutions with one common rule across
M24 and X36. At most two anchors per scale will be frozen before any H=3
tracking score is opened.

## Decision gate

The next method stage is authorized only if the current-only scan finds, at
both scales, a safe sender substitution with material local net support and a
non-trivial sender-novelty term. Frozen paired H=3 actions must then show at
least `5%` mean tracking improvement at both M24 and X36, with no worse sensor
or formation tail. If the source gate fails, the method returns to scene/action
design. If the H=3 gate fails, no GNN or learned selector is trained on that
action family.

This is development evidence, not a validation claim.
