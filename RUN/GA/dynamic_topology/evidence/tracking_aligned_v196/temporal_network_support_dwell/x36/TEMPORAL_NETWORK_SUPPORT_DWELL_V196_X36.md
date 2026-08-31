# V196 X36 temporal network-support dwell replay

| t | V99 proposal | V194 release | Current network | 2-page dwell | 3-page dwell |
|--:|:--|:--|:--|:--|:--|
| 72 | `[1 2 4 5]` | `[2 5]` | `[2 5]` | `[2 5]` | `[2 5]` |
| 73 | `[1 3 4 5]` | `[3 5]` | `[3 5]` | `3` | `3` |
| 74 | `[1 3 4 5]` | `3` | `3` | `[]` | `[]` |

- First-page desired release preserved (W=2): `1`
- First-page desired release preserved (W=3): `1`
- Later current-network releases: `3`
- Later W=2 releases: `1`
- Later W=3 releases: `1`

## Per-formation unsupported-entry count

- t=72 current `[0 1 NaN 0 1 NaN]`, W=2 `[0 1 NaN 0 1 NaN]`, W=3 `[0 1 NaN 0 1 NaN]`
- t=73 current `[0 NaN 1 0 1 NaN]`, W=2 `[0 NaN 1 0 0 NaN]`, W=3 `[0 NaN 1 0 0 NaN]`
- t=74 current `[0 NaN 1 0 0 NaN]`, W=2 `[0 NaN 0 0 0 NaN]`, W=3 `[0 NaN 0 0 0 NaN]`

## Evidence boundary

V196 replays the observable local-posterior sequence already visited by the paired V194 arm. For each current contributing endpoint and entered label, it takes the maximum detection-association mass over the current and preceding one or two replay pages. The first page has no pre-intervention support history and therefore falls back to current support. No truth, future measurement or tracking outcome enters the release decision.
