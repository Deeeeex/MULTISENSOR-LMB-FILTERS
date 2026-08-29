# V148 M24 finding: deleting reference-cover labels closes the sender-only rule

## Decision

V148 fails its preregistered M24 gate and stops before X36.  Its
intervention-window E-OSPA gain is `+1.644%`, below the required `+5%`.
The result is repository-only and must not be promoted to the main progress
document.

The failure also exposes a semantic defect in the initial byte projector.
A label that is present in the same-edge reference payload is not optional
merely because its current existence and association scores are below the
decision thresholds.  Removing it changes the support supplied to the
receiver and couples the proposed W/R role choice to the separate
missing-label fusion approximation.  A reportable successor must preserve the
complete reference label cover before asking whether W or R should provide
the density for a shared label.

## Frozen M24 result

- Source commit: `aec964c`.
- Preset / seed / anchor / action:
  `m24-formation-fov` / `1601` / `95` / `25`.
- Intervention / full / mature E-OSPA gains:
  `+1.644% / +5.533% / +6.159%`.
- Minimum sensor / formation gains: `+0.790% / +3.355%`.
- Rejoined relay match: `100%`.
- Attempted-byte delta: `-0.785%`; auxiliary bytes: `0`.
- Hybrid working edges: `72`; selected W/R/rescue labels:
  `825 / 212 / 5`.
- Optional labels deleted / full-reference fallbacks: `16 / 8`.

Against V143's whole-W working pages, V148 changes the intervention gain from
`+1.882%` to `+1.644%` (`-0.238` percentage points), while the full and mature
gains are essentially unchanged (`+5.554%` to `+5.533%`, and `+6.159%` in
both cases).  The extra `0.063` percentage points of byte saving therefore do
not compensate for the short-window loss.

## Time-localized attribution

The hybrid projector acts on only three continuation pages:

| Absolute time | Hybrid edges | Deleted labels | Full-R fallbacks | Rescue labels | Mean E-OSPA delta, V148 minus V143 |
|--:|--:|--:|--:|--:|--:|
| 96 | 36 | 0 | 3 | 0 | `-0.000808` |
| 98 | 27 | 7 | 5 | 5 | `+0.678044` |
| 100 | 9 | 9 | 0 | 0 | `+0.335508` |

Positive delta means V148 is worse.  The two pages that delete labels account
for `99.929%` of the signed V148--V143 intervention gap.  On time 96, where
36 edges already use the label-wise construction but no label is deleted,
the two methods are numerically indistinguishable at network scale.  This
does not prove that sender-only W/R selection is useful; it shows that the
observed regression cannot be attributed cleanly to label-role selection
until reference support deletion is removed.

## Fusion-semantics boundary

The registered continuation uses `fov-aware-censored` missing-label fusion.
That rule gives observable absence a finite censored-existence
representation and excludes geometrically uninformative absence.  It is an
explicit robustness approximation for different FoVs, not exact set-density
LMB-KLA, and it does not make deliberate payload deletion information-free.
Payload selection and missing-label robustness must therefore be evaluated as
separate method components.

## Next method decision

The next bounded probe changes both the safety invariant and the value
coordinate:

1. Every label in the same-edge R payload is mandatory.  A shared label may
   carry either its complete W or complete R Gaussian-mixture Bernoulli
   density, but it may not disappear.  Only W-only labels may be omitted.
2. If W choices exceed the R byte cap, low-value shared labels revert to their
   R object before any W-only extra is removed; if the payload still cannot
   fit, the edge sends unchanged R.
3. Value is receiver-relative.  A frozen headroom probe may inspect the
   receiver's current posterior and virtual label-wise fusion to decide which
   sender density repairs receiver support without crossing a protected
   existence boundary.  This privileged state access is teacher evidence,
   not yet a deployable communication rule.
4. Only joint M24/X36 headroom above the existing gates authorizes an
   observable request/synopsis design or a GNN.  The synopsis must then be
   explicitly transmitted and charged.  If the receiver-relative safe action
   family lacks joint headroom, label-role payload construction is closed and
   the next method must reallocate route timing or physical message budget.

