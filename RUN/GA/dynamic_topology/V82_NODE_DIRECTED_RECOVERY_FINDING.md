# V82 node-directed recovery finding

## Decision

Reject node-directed dominant-input damping as the missing cross-scale
recovery mechanism.  V82 does not pass its one-step headroom gate and no
multi-round or tracking run is opened from this branch.

## Result

V82 decomposes the V77 centered perturbation energy exactly by receiver,
nominates at most the two receivers whose contribution grows most after an
ordinary reference-recovery step, and compares reference recovery, the frozen
small global-balance actions, and sparse dominant-input damping at those
receivers.

| Scale / route | Pulse | Best next action | Next energy | Contracts |
|:--|--:|:--|--:|:--:|
| M24 historical | 0.00503634 | reference | 0.00557678 | no |
| M24 receiver-aligned | 0.00400088 | reference | 0.00421286 | no |
| X36 historical | 0.00179825 | global balance 0.02 | 0.00143671 | yes |
| X36 receiver-aligned | 0.00198748 | global balance 0.05 | 0.00145866 | yes |

For M24, every nominated-node damping action is worse than ordinary recovery;
even the least invasive factor 0.95 raises the centered energy.  For X36, the
selected actions are exactly the small global-balance points already exposed
by V80.  Node nomination therefore explains where the reference transition
concentrates the perturbation, but adds no useful action beyond V80.

## Method implication

V79--V82 have now tested global source replacement, support-preserving global
weight balancing, intervention-local soft return, and propagation-node
dominant-input damping.  The repeated M24 failure is evidence that the current
hard requirement of monotone V77 centered-energy recovery is not a productive
surrogate for the requested tracking objective.  V74 had already shown that
single-round internal disagreement can disagree with tracking value.

The next experiment therefore stops refining this recovery proxy.  It opens a
task-aligned **route-and-trust headroom** question: after choosing an
alternative cross-formation sender, can a fixed-message action obtain material
M24 and X36 H=3 tracking gains by allocating it more than the frozen 0.05
residual weight, funded from receiver self-weight while preserving the 0.70
within-formation backbone?  Only if that compact action family has clear
paired-return headroom will a variable-size observable value model be trained
to select route, trust, and fallback online.

## Evidence boundary

The V82 result is source-side development evidence.  It reads no truth, future
measurement, tracking outcome, or learned prediction.  It supports rejecting
the tested recovery action family; it does not establish tracking performance
for any successor method.
