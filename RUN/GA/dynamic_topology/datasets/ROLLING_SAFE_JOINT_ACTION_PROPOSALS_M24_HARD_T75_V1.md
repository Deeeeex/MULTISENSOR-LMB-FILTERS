# Rolling-safe joint-action proposal dataset: M24

- Generated: 2026-07-29 09:49:36
- Contract: `rolling-safe-joint-action-proposals-m24-v1`
- Proposal bank: `rolling-safe-joint-action-proposal-bank-v1`
- Source state SHA-256: `ca08e2eec2c7a3e63ceca8b3ede292cea9cdb690666f58c64be8ec4df83866c9`
- Generation commit: `211aac475d2cfe3eeba85ae8d71bc8604379a938`
- Generation tracked worktree dirty: `0`
- Seeds: `[7 11 17 19 23 29]`
- Times: `[75 76 77]`
- Frozen proposal attempts per state: `28`
- Distinct candidates min/mean/max: `12 / 18.94 / 23`
- Selectable candidates min/mean/max: `11 / 18.78 / 23`
- Joint feature dimension: `880`
- Truth used: `0`
- Future outcome used: `0`
- Return evaluated: `0`
- Continuation posteriors stored: `1`
- Evidence boundary: This artifact verifies truth-free distinct safe candidate construction and joint state-action featurization on the frozen M24 development states. Truth-free predecision posteriors are stored only to restart paired counterfactual continuations. It contains no candidate tracking returns and does not establish top-K coverage, oracle gain, critic accuracy, X36 support or end-to-end communication savings.

## State blocks

| Seed | Time | Distinct | Duplicates | Selectable | Unavailable | Reference index | Reference repaired | Action codes |
|--:|--:|--:|--:|--:|--:|--:|--:|:--|
| 7 | 75 | 21 | 7 | 21 | 0 | 1 | 0 | `24-60-61-62-63-64-65-66-67-68-69-70-71-72-73-74-78-80-82-83-84` |
| 7 | 76 | 19 | 6 | 19 | 3 | 1 | 0 | `24-60-61-62-66-67-68-69-70-71-72-73-74-75-76-77-80-82-83` |
| 7 | 77 | 17 | 4 | 16 | 7 | 1 | 1 | `24-61-62-64-65-67-68-70-71-73-74-77-80-82-83-85-86` |
| 11 | 75 | 23 | 5 | 23 | 0 | 1 | 0 | `24-60-61-62-63-64-65-66-67-68-69-70-71-72-73-74-75-76-77-78-80-82-83` |
| 11 | 76 | 23 | 5 | 23 | 0 | 1 | 0 | `24-60-61-62-63-64-65-66-67-68-69-70-71-72-73-74-75-76-77-80-82-83-84` |
| 11 | 77 | 16 | 7 | 16 | 5 | 1 | 0 | `24-60-61-62-63-64-65-68-71-72-76-77-80-82-83-85` |
| 17 | 75 | 16 | 9 | 16 | 3 | 1 | 0 | `24-60-61-62-63-64-65-70-71-72-73-74-78-80-82-84` |
| 17 | 76 | 22 | 6 | 22 | 0 | 1 | 0 | `24-60-61-62-63-64-65-67-68-69-70-71-72-73-74-75-76-77-80-82-83-84` |
| 17 | 77 | 12 | 5 | 11 | 11 | 1 | 1 | `24-61-62-70-71-73-74-80-82-83-85-86` |
| 19 | 75 | 17 | 8 | 17 | 3 | 1 | 0 | `24-61-62-63-64-65-69-70-71-73-74-77-78-80-82-83-84` |
| 19 | 76 | 17 | 8 | 17 | 3 | 1 | 0 | `24-60-61-62-63-64-65-69-70-71-72-73-74-75-80-82-83` |
| 19 | 77 | 20 | 4 | 20 | 4 | 1 | 0 | `24-60-61-62-63-64-65-69-70-71-72-73-74-76-77-80-82-83-84-85` |
| 23 | 75 | 20 | 8 | 20 | 0 | 1 | 0 | `24-61-62-63-64-65-66-67-68-69-70-71-72-73-74-78-80-82-83-84` |
| 23 | 76 | 23 | 5 | 23 | 0 | 1 | 0 | `24-60-61-62-63-64-65-66-67-68-69-70-71-72-73-74-75-76-77-80-82-83-84` |
| 23 | 77 | 18 | 3 | 17 | 7 | 1 | 1 | `24-61-62-64-65-67-68-70-71-73-74-77-80-82-83-84-85-86` |
| 29 | 75 | 21 | 7 | 21 | 0 | 1 | 0 | `24-60-61-62-63-64-65-66-67-68-69-70-71-72-73-74-77-78-80-82-83` |
| 29 | 76 | 21 | 5 | 21 | 2 | 1 | 0 | `24-60-61-62-63-64-65-66-67-68-69-70-72-74-75-76-77-80-82-83-84` |
| 29 | 77 | 15 | 4 | 15 | 9 | 1 | 0 | `24-61-62-64-65-70-71-73-74-77-80-82-83-85-86` |

## Aggregate construction readout

- Total distinct state-actions: `341`
- Total duplicate proposals removed: `106`
- Total unavailable proposals: `57`
- Candidate cap respected on every state: `1`
- Candidate tracking-return gate evaluated: `0`
