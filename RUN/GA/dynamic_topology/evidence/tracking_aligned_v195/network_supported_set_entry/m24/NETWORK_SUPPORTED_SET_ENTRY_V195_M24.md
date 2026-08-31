# V195 M24 network-supported set-entry replay

| t | V99 proposal | V194 runtime release | Receiver-only replay | Receiver-or-peer replay |
|--:|:--|:--|:--|:--|
| 104 | `[1 3 4]` | `4` | `4` | `4` |
| 105 | `[1 2 3]` | `[1 2]` | `[1 2]` | `[]` |
| 106 | `[1 2 3]` | `[1 2]` | `[1 2]` | `[1 2]` |

- First-page desired release preserved: `1`
- Later receiver-only releases: `4`
- Later receiver-or-peer releases: `2`

## Per-formation risk

- t=104 receiver count `[0 NaN 0 1]`, network count `[0 NaN 0 1]`
  receiver risk `[0 NaN 0 0.054916]`, network risk `[0 NaN 0 0.054916]`
- t=105 receiver count `[1 3 0 NaN]`, network count `[0 0 0 NaN]`
  receiver risk `[0.20859 0.28313 0 NaN]`, network risk `[0 0 0 NaN]`
- t=106 receiver count `[8 3 0 NaN]`, network count `[1 1 0 NaN]`
  receiver risk `[1.299 0.30462 0 NaN]`, network risk `[0.16745 0.10346 0 NaN]`

## Evidence boundary

V195 replays the current-observable V194 states already visited in the paired recursive screen. It changes only the support definition: an entering label is supported when the receiver or an active cross-formation reference sender has current measurement association. No tracking outcome, truth or future information enters the replay.
