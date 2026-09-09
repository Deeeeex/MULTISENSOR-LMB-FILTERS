# Joint current-detection admission

Start from the clean `09bd9c62` archive. The shared range-detection screen
failed three of its eight continuation comparisons. Its diagnostic case
showed that a nearby detection could carry zero positive mark support, but
did not show that changing admission repairs a recursive trajectory.
The earlier conditional no-mark GCE ablation is also already known: it
worsened the original nine-segment mean OSPA by about 0.506 m. Both failures
remain part of this development history.

This experiment changes only positive current-ratio admission. Let r_plus
be the executed local posterior existence and W_m the local association
weight conditional on this label existing. Let a=sum_{m>0} W_m on a current
sensing opportunity, and zero otherwise. The single primary rule is

    g_positive = r_plus * a.

It is the approximate posterior probability of the joint event that this
label exists and is associated with a current detection. The detection
mark already enters the local likelihood and association update. There is
no second positive-mark multiplier in this rule. Conditional no-mark
admission uses a instead; this distinction reduces admission from uncertain
labels. It does not make inherited belief independent or calibrate LBP.
Strong false labels can still receive substantial support.

Retain nominal pD=0.9, negative support (1-a)*pD/(2-pD), current-opportunity
reset, inherited beta, eligible-source matching and absence semantics,
source curvature rejection, aggregate fallback, Gaussian integration,
extraction and original observations/radio draws. No probability threshold,
power, clipping rule, fitted coefficient, range model or history state is
added. A recursive implementation would reuse the existing support field
in the 352-byte Gaussian component packet.

## Fixed-input screen, frozen before new scores

Use every original nine V2V4Real development segment and the five previously
exposed V2X-Real validation segments, both link conditions. Use the saved
nominal GCE and nominal Guarded Scalar recursive states separately: 56
source runs. All 43 V2V and 19 V2X segments have already been exposed during
this project; no part of this screen is an independent test. The diagnosed
v2xt_0001 case is excluded from method selection.

At each source run's visited fusion inputs compare the complete two-factor
table, with exactly one primary and no secondary promotion:

| Rule | Conditional association | Joint existence and association |
|---|---|---|
| Retain the old mark multiplier | original | joint_mark: r_plus * old support |
| Omit the extra mark multiplier | conditional: a | joint: r_plus * a (primary) |

Reconstruct all Gaussian ratios from recorded local moments, recompute
the guard and complete existence/spatial distribution, and extract the
set using the unchanged MAP/cardinality and geometric domain rule. On a
frame without a received message retain the actual local output. Never
feed these alternative outputs into the next frame. Verify exact original
set parity and score parity before interpreting alternatives. Verify the
joint-probability identity against finite enumeration and the saved raw W
where available. Record the limits of older files without raw W.

Advance the joint primary to full recursion only if, on the original GCE
states, its two-link sequence-macro OSPA is strictly lower than both original
and conditional admission in each dataset (four comparisons). Complete all
56 source runs and report every rule, both backends, every sequence/link,
miss/false/localization costs, output counts and changes. Guarded Scalar
states and joint_mark are diagnostic controls and cannot select another
rule. If any comparison fails, close this exact family without a new gate
coefficient or a subset chosen from favorable sequences. A passed fixed-
input screen would establish only a reason to run the full recursion.

## Conditional recursive continuation

If the screen passes, freeze the native implementation before running it.
First require original GCE and conditional no-mark parity on development
0000 and the complete analytic/packet/source checks. Then run the joint
rule under both GCE and Guarded Scalar on all 14 screen segments, with
both links. Include original GCE, original No-age, and conditional no-mark
GCE as declared controls, reusing complete audited native outputs when
available and running the missing control cells. Each new arm follows its
own full recursion. Require at least 1% lower two-link sequence-macro OSPA
than original GCE in each dataset, and strictly lower OSPA than the matched
joint Guarded Scalar, original No-age and conditional no-mark GCE in each.
Failure closes the family; no secondary can replace the primary.

Only after these recursive gates pass may the unchanged rule extend to the
remaining 34 V2V and 14 V2X segments with the matched controls. Report all
outcomes and recording/date sensitivity; already exposed data cannot supply
a fresh validation claim. No paper promotion follows from this screen.

Protect source, previous results and input hashes before and after the
screen. Preserve any failed verification and its minimal repair separately.
Do not overwrite a frozen experiment, alter previous methods, or claim
new physical communication costs from fixed-input substitutions.
