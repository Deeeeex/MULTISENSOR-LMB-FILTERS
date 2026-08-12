# V120 finding: reversing the whole carrier redistributes tail risk

## Result

V120 compares two fixed-orientation abstention controls with reciprocal
four-page carrier switches.  All pages retain 60 selected messages, the same
fusion-weight multiset, physicality, row stochasticity and rolling sensor and
formation B3.  The fixed controls reuse paired V110/V113 outcomes; only the two
switch trajectories are newly executed.

| Carrier schedule | Mean E-OSPA | Gain vs best full | vs fixed CCW | vs fixed CW | Mature min. | Min. formation | Terminal min. formation | Byte saving |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|
| fixed CCW | 79.517797 | +2.794% | -- | -- | -- | -- | -- | +4.433% |
| fixed CW | 78.479689 | +4.063% | -- | -- | -- | -- | -- | +3.330% |
| CW pages 1--4, CCW pages 5--8 | 78.581372 | +3.939% | +1.178% | -0.130% | +2.770% | -1.203% | -5.831% | +4.008% |
| CCW pages 1--4, CW pages 5--8 | 79.485999 | +2.833% | +0.040% | -1.282% | -0.997% | -6.307% | -8.726% | +4.078% |

No arm passes.  The targeted CW-to-CCW switch is better than fixed CCW and the
reciprocal switch, but it remains worse than fixed CW and reaches only
`+3.939%` versus the better full-payload baseline.  Its mature-page minimum is
also only `+2.770%`.

## Mechanism

The target switch partially repairs F6 but does not create a net improvement.
Relative to CW full, its formation gains are
`[-1.073, 0.979, 10.620, 4.576, 9.913, -1.203]%`.  Fixed CW abstention had
approximately zero F1 loss and `-1.495%` F6 loss.  Reversing the carrier after
page 4 therefore reduces the F6 window loss by about `0.292` percentage points
but introduces a `1.073%` F1 loss.  At the terminal page, F6 still loses
`5.831%`.  The action moves the weak tail around the ring instead of removing
it.

The reciprocal schedule ends only `0.040%` above fixed CCW, despite spending
its last four pages clockwise.  This shows that the posterior state created by
the first half of the horizon persists; changing the later adjacency cannot
simply undo earlier recursive fusion.  A carrier controller therefore cannot
be trained as a memoryless choice between two global orientations.

## Decision

The binary whole-ring orientation-switch family is closed on this opened
anchor.  A further switch-time scan would interpolate between the two fixed
endpoints without addressing their coupled F1/F6 risk.  The next action should
break that coupling: retain exact budget and rolling connectivity, but allow
different formation segments to use different influence directions in the
same page.  Before tracking, V121 must enumerate the structurally valid mixed
carriers and establish that such a carrier can independently route the F1 and
F6 return paths.  Only a positive bounded mixed-carrier oracle would justify
learning segment-level edge values.

V120 is privileged development evidence at X36 seed 211, t=72, H=8.  It is not
deployable, validation or generalization evidence.
