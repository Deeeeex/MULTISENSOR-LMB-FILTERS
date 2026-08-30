# V187 formation-coordinator aggregation preflight

- Gate passed: `1`
- Same action as uncapped selector: `1`
- Coordinator action: `[31,24] <- 31`
- Synopsis bytes per label: `64`
- Side-channel bytes: `139680 -> 79488` (`+43.093%` saving)
- Coordinator synopsis / request / response bytes: `51952 / 7040 / 20496`

| Receiver | E-OSPA gain | RMSE gain |
|--:|--:|--:|
| 25 | +0.227196 | +2.029721 |
| 26 | +0.177008 | +1.472425 |
| 27 | +0.136269 | +1.721364 |
| 28 | +0.161047 | +1.348901 |
| 29 | +0.191406 | +1.773849 |
| 30 | +0.154169 | +1.514527 |

## Evidence boundary

V187 replays the opened V180 X36 seed-211 t=79 state with ideal delivery. One formation coordinator receives each source inventory once and five peer-receiver summaries once. Every per-label message is charged as a 64-byte position-moment synopsis; selection consumes only the reconstructed synopsis objects. Truth is read after selection for immediate E-OSPA/RMSE rejection. A pass authorizes one recursive development run only.
