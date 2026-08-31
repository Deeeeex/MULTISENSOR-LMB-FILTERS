# V195 X36 network-supported set-entry replay

| t | V99 proposal | V194 runtime release | Receiver-only replay | Receiver-or-peer replay |
|--:|:--|:--|:--|:--|
| 72 | `[1 2 4 5]` | `[2 5]` | `[2 5]` | `[2 5]` |
| 73 | `[1 3 4 5]` | `[3 5]` | `[3 5]` | `[3 5]` |
| 74 | `[1 3 4 5]` | `3` | `3` | `3` |

- First-page desired release preserved: `1`
- Later receiver-only releases: `3`
- Later receiver-or-peer releases: `3`

## Per-formation risk

- t=72 receiver count `[0 2 NaN 0 2 NaN]`, network count `[0 1 NaN 0 1 NaN]`
  receiver risk `[0 0.12268 NaN 0 0.12059 NaN]`, network risk `[0 0.062987 NaN 0 0.060337 NaN]`
- t=73 receiver count `[0 NaN 2 0 2 NaN]`, network count `[0 NaN 1 0 1 NaN]`
  receiver risk `[0 NaN 0.12563 0 0.11447 NaN]`, network risk `[0 NaN 0.062183 0 0.057915 NaN]`
- t=74 receiver count `[0 NaN 3 0 0 NaN]`, network count `[0 NaN 1 0 0 NaN]`
  receiver risk `[0 NaN 0.13321 0 0 NaN]`, network risk `[0 NaN 0.021703 0 0 NaN]`

## Evidence boundary

V195 replays the current-observable V194 states already visited in the paired recursive screen. It changes only the support definition: an entering label is supported when the receiver or an active cross-formation reference sender has current measurement association. No tracking outcome, truth or future information enters the replay.
