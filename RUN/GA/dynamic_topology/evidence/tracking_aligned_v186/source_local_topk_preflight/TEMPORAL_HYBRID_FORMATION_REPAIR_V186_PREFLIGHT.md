# V186 source-local Top-K advertisement preflight

- Gate passed: `0`
- Source-local label cap: `4`
- Same action as uncapped selector: `0`
- Uncapped action: `[31,24] <- 31`
- Capped action: `[0,0] <- 0`
- Synopsis bytes: `118992 -> 23520` (`+80.234%` saving)
- Total side-channel bytes: `139680 -> 23520` (`+83.162%` saving)

| Receiver | Capped E-OSPA gain | Capped RMSE gain |
|--:|--:|--:|
| 25 | +0.000000 | +0.000000 |
| 26 | +0.000000 | +0.000000 |
| 27 | +0.000000 | +0.000000 |
| 28 | +0.000000 | +0.000000 |
| 29 | +0.000000 | +0.000000 |
| 30 | +0.000000 | +0.000000 |

## Evidence boundary

V186 preflight replays the opened V180 X36 seed-211 t=79 pre-side-channel state with ideal delivery. It compares the frozen uncapped formation selector against a source-local Top-4 synopsis ranked only by current posterior position uncertainty. Truth is read after selection for immediate E-OSPA/RMSE rejection. A pass authorizes one recursive development run only; it is not validation.
