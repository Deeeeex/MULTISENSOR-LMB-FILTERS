# V196 M24 temporal network-support dwell replay

| t | V99 proposal | V194 release | Current network | 2-page dwell | 3-page dwell |
|--:|:--|:--|:--|:--|:--|
| 104 | `[1 3 4]` | `4` | `4` | `4` | `4` |
| 105 | `[1 2 3]` | `[1 2]` | `[]` | `[]` | `[]` |
| 106 | `[1 2 3]` | `[1 2]` | `[1 2]` | `[]` | `[]` |

- First-page desired release preserved (W=2): `1`
- First-page desired release preserved (W=3): `1`
- Later current-network releases: `2`
- Later W=2 releases: `0`
- Later W=3 releases: `0`

## Per-formation unsupported-entry count

- t=104 current `[0 NaN 0 1]`, W=2 `[0 NaN 0 1]`, W=3 `[0 NaN 0 1]`
- t=105 current `[0 0 0 NaN]`, W=2 `[0 0 0 NaN]`, W=3 `[0 0 0 NaN]`
- t=106 current `[1 1 0 NaN]`, W=2 `[0 0 0 NaN]`, W=3 `[0 0 0 NaN]`

## Evidence boundary

V196 replays the observable local-posterior sequence already visited by the paired V194 arm. For each current contributing endpoint and entered label, it takes the maximum detection-association mass over the current and preceding one or two replay pages. The first page has no pre-intervention support history and therefore falls back to current support. No truth, future measurement or tracking outcome enters the release decision.
