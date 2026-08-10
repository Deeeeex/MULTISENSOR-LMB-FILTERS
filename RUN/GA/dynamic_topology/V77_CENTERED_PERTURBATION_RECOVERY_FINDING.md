# V77 centered perturbation-recovery finding

V77 corrects the V76 reference-frame error, but the current reference recovery
schedule still fails the frozen centered-mode condition on every historical
and aligned route.

| Route | Centered energy by round | Terminal change | Certificate |
|:--|:--|--:|:--:|
| M24 V71/V72 | `0.005036, 0.005577, 0.004053` | `-19.523%` | fail: round-2 amplification |
| M24 V73 | `0.004001, 0.004213, 0.004165` | `+4.100%` | fail |
| X36 V71/V72 f4 | `0.001798, 0.001447, 0.002897` | `+61.123%` | fail |
| X36 V73 f4 | `0.001987, 0.001476, 0.002900` | `+45.890%` | fail |

Common-mode energy grows in all four replays.  V77 does not penalize that
growth: it represents a candidate-induced change becoming shared across the
network.  The failed condition comes from the centered component itself, so
it cannot be dismissed as useful information merely moving away from the
reference arm.

The component split identifies the mechanism.  Centered existence energy is
one to two orders of magnitude larger than centered spatial energy at the
third round.  For aligned X36 it changes from `0.001895688` to `0.001420873`
and then rebounds to `0.002781713`; the spatial term changes from
`0.000091794` to `0.000054724` and then to `0.000117823`.  M24 shows the same
non-monotone pattern.  Thus V75's direct spatial-conflict gate and V77's
existence-dominated recovery debt address different failure modes.

The current result rejects the fixed policy "one direct-safe pulse, then two
reference rounds" as a recovery certificate.  It does not reject the V75-safe
first-round route or prove tracking harm.  The next source-only question is
whether the existing action space contains a better two-round recovery
schedule.  V78 should keep the first pulse fixed, enumerate reference versus
the same direct-safe route in each of the next two rounds, and minimize peak
centered amplification before terminal centered energy.  If none of the four
schedules removes the rebound, recovery requires a broader balanced route or
weight action rather than another scalar threshold.

V77 uses deterministic current-link reliability and no prediction,
measurement, future link, truth, packet draw, route execution, or tracking
outcome.  It is opened-anchor mechanism evidence only.
