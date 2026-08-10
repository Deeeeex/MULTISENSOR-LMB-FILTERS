# V78 recovery-sequence search finding

V78 closes the duration-only branch of the current recovery design.  None of
the four schedules available in the binary action space (`CRR`, `CRC`, `CCR`,
or `CCC`) makes the centered perturbation monotonically non-increasing on
either M24 or X36, for either the historical route or the receiver-aligned
route.

| Case | Best eligible schedule | Peak factor | Terminal factor | Monotone |
|:--|:--:|--:|--:|:--:|
| M24 historical | `CRR` | 1.107 | 0.805 | no |
| M24 aligned | `CRR` | 1.053 | 1.041 | no |
| X36 historical | `CRR` | 1.611 | 1.611 | no |
| X36 aligned | `CRR` | 1.459 | 1.459 | no |

The ranking is consistent across scales: one candidate pulse followed by two
reference rounds is already the least amplifying schedule.  Re-applying the
candidate in round 2 or 3 increases both peak and terminal centered energy.
For M24, `CCR` and `CCC` also fail the frozen V75 replacement-innovation gate
after the first candidate pulse.  For X36 the repeated candidate remains
inside that direct spatial trust region, yet its centered recovery becomes
substantially worse.  Direct replacement safety and network recovery are
therefore distinct conditions.

This result rejects persistence tuning as the missing control mechanism.  It
does not reject the first V75-safe pulse and does not establish tracking harm.
The current action space can only choose between the same candidate matrix and
the same reference matrix; it cannot alter the contraction properties of the
recovery operator.  V79 should instead construct a recovery matrix that keeps
the directed-message count and per-receiver weight budget fixed while reducing
centered amplification.  A natural first candidate is to replace the
within-formation index star during recovery with a physical balanced cycle:
the self, dominant, and residual components then become weighted permutation
matrices before reliability scaling, making the nominal fusion matrix doubly
stochastic and directly controlling the centered consensus mode.

V78 is an opened-anchor, source-only mechanism screen.  It uses deterministic
current-link reliability and no prediction, measurement, future link, truth,
packet draw, route execution, tracking outcome, or model training.
