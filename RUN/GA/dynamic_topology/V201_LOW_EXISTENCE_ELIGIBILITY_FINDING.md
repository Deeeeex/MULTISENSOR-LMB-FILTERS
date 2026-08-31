# V201 low-existence teacher eligibility finding

## Finding

The first X36 `seed=211, t=72, H=8` F2+F3 recursive teacher run did not
evaluate the planned F3 mechanism.  Only the established F2 action at `t=72`
was applied.  The F3 actions scheduled at `t=76`, `t=78`, and `t=79` matched
zero candidates and therefore left the recursive tracking state unchanged.

| Scheduled page | Formation | Source | Label | Candidate bank | Forced match | Fused receiver existence |
|--:|--:|--:|:--|--:|--:|--:|
| 76 | 3 | 23 | `[19,13]` | 773 | 0 | 0.3280 |
| 78 | 3 | 23 | `[19,13]` | 894 | 0 | 0.3215 |
| 79 | 3 | 22 | `[19,13]` | 839 | 0 | 0.3183 |

The source existence was approximately one on all three pages, and all six
F3 receivers contained the active label.  The exclusion came from the
candidate enumerator's default `receiverPresenceThreshold=0.5`.  This gate is
appropriate for the current conservative online proposal builder, but it is
not an eligibility condition for an explicitly scheduled, MAP-sensitive
teacher action.  The cache's active-label threshold is `0.01`.

## Observed run

Because the three F3 actions were not applied, the tracking outcome exactly
matches the earlier F2-only V199 mechanism result: E-OSPA `78.274634`, RMSE
`58.204481`, and consensus gain `11.348%` relative to the static reference.
The attempted-byte saving fell to `4.451%` because the unused light-synopsis
pages were still charged.  These numbers are an eligibility-failure record,
not evidence for or against the F3 label-KLA action.

## Correction and boundary

Only the forced source-label teacher branch now enumerates any label active
at all receivers (`r >= 0.01`).  The automatic online proposal path retains
the default `r >= 0.5` threshold.  This lets the corrected rerun answer the
bounded mechanism question without silently changing the deployable policy.
It does not validate an online selector or justify updating the paper-facing
current-best result.
