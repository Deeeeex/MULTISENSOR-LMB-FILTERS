# V184 formation-coordinated posterior-repair diagnostic

- Formation: `5`
- Common-action support requirement: `>= 4 nodes`
- Minimum safety probability: `0.600`
- Minimum observable risk reduction: `-0.050`
- Action semantics: replace one already-present complete single-label posterior; both existence and spatial density may change.
- Eligible / truth-safe contexts: `6 / 6`

| State | t | Common / eligible | Selected label <- source | Support | Min P | Min risk | Rescue score | E-OSPA / RMSE gain | Min node E / R | Harmful |
|:--|--:|:--|:--|:--|--:|--:|--:|--:|--:|--:|
| V166 static state | 78 | 73 / 10 | `[7,6] <- 15` | 6/6 | 0.888 | +0.026 | 0.3044 | +42.889 / +1.476 | +7.141 / +0.200 | 0 |
| V166 static state | 79 | 67 / 8 | `[7,6] <- 14` | 6/6 | 0.853 | +0.027 | 0.2645 | +42.867 / +1.363 | +7.138 / +0.138 | 0 |
| V176 first rollout | 78 | 68 / 9 | `[31,24] <- 32` | 6/6 | 0.854 | +0.001 | 0.3542 | +0.542 / +3.321 | +0.048 / +0.178 | 0 |
| V176 first rollout | 79 | 71 / 6 | `[7,6] <- 17` | 6/6 | 0.839 | +0.027 | 0.3158 | +44.896 / +0.755 | +6.462 / +0.041 | 0 |
| V180 second rollout | 78 | 68 / 9 | `[31,24] <- 32` | 6/6 | 0.854 | +0.001 | 0.3542 | +0.542 / +3.321 | +0.048 / +0.178 | 0 |
| V180 second rollout | 79 | 71 / 3 | `[31,24] <- 31` | 4/6 | 0.602 | -0.006 | 0.2934 | +1.047 / +9.861 | +0.136 / +1.349 | 0 |

## Per-node truth readout

- V166 static state, t=78, `[7,6] <- 15`

| Receiver | E-OSPA gain | RMSE gain |
|--:|--:|--:|
| 25 | +7.151625 | +0.253702 |
| 26 | +7.143787 | +0.199676 |
| 27 | +7.153948 | +0.243142 |
| 28 | +7.151264 | +0.251267 |
| 29 | +7.140713 | +0.269463 |
| 30 | +7.147844 | +0.258687 |

- V166 static state, t=79, `[7,6] <- 14`

| Receiver | E-OSPA gain | RMSE gain |
|--:|--:|--:|
| 25 | +7.146036 | +0.137764 |
| 26 | +7.144716 | +0.249757 |
| 27 | +7.144874 | +0.255968 |
| 28 | +7.146734 | +0.236689 |
| 29 | +7.138242 | +0.248792 |
| 30 | +7.146848 | +0.234151 |

- V176 first rollout, t=78, `[31,24] <- 32`

| Receiver | E-OSPA gain | RMSE gain |
|--:|--:|--:|
| 25 | +0.047685 | +0.177678 |
| 26 | +0.102372 | +0.600781 |
| 27 | +0.087057 | +0.580365 |
| 28 | +0.104941 | +0.706643 |
| 29 | +0.108274 | +0.668517 |
| 30 | +0.091982 | +0.587089 |

- V176 first rollout, t=79, `[7,6] <- 17`

| Receiver | E-OSPA gain | RMSE gain |
|--:|--:|--:|
| 25 | +6.461624 | +0.041444 |
| 26 | +7.127736 | +0.156301 |
| 27 | +7.129062 | +0.161728 |
| 28 | +8.063065 | +0.128811 |
| 29 | +8.051317 | +0.139894 |
| 30 | +8.062791 | +0.127280 |

- V180 second rollout, t=78, `[31,24] <- 32`

| Receiver | E-OSPA gain | RMSE gain |
|--:|--:|--:|
| 25 | +0.047685 | +0.177678 |
| 26 | +0.102372 | +0.600781 |
| 27 | +0.087057 | +0.580365 |
| 28 | +0.104941 | +0.706643 |
| 29 | +0.108274 | +0.668517 |
| 30 | +0.091982 | +0.587089 |

- V180 second rollout, t=79, `[31,24] <- 31`

| Receiver | E-OSPA gain | RMSE gain |
|--:|--:|--:|
| 25 | +0.227196 | +2.029721 |
| 26 | +0.177008 | +1.472425 |
| 27 | +0.136269 | +1.721364 |
| 28 | +0.161047 | +1.348901 |
| 29 | +0.191406 | +1.773849 |
| 30 | +0.154169 | +1.514527 |

## Decision boundary

V184 is a fixed-rule, same-seed mechanism diagnostic over opened V166/V176/V180 X36 states. Selection consumes only current posterior-derived features and a V181 safety model; truth is read only after selection. V180 t=78 is intentionally retained in the table as a duplicate-state consistency check, not as independent evidence. Even a clean result authorizes only a frozen recursive development probe, not validation or a paper-facing claim.
