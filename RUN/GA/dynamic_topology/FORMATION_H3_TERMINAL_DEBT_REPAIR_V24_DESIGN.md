# Formation H=3 terminal debt-repair grid v24

## Question

v23 shows that a second action can reduce debt but cannot produce a strong,
strictly feasible sequence while the third action is fixed to reference.  v24
opens only the third action and asks whether the existing three-step mode
family contains any teacher sequence with all six targets nonnegative and at
least `3%` mean tracking gain.

## Frozen prefixes

| First step | Second step | Role | v23 terminal debt |
|:--|:--|:--|:--|
| `[1,1,2,2]` | `[1,4,1,1]` | closest to the strong boundary | sensor tail and two byte targets |
| `[1,4,3,1]` | `[1,1,4,4]` | minimum high-return consensus debt | consensus and delivered bytes |
| `[1,4,3,1]` | `[1,1,2,4]` | minimum high-return delivered-byte debt | consensus and delivered bytes |
| `[1,4,3,1]` | `[1,1,1,4]` | maximum high-return headroom | consensus and delivered bytes |

Each prefix is crossed with the same structured 18-vector grid used in v23:
formation 1 remains reference, formation 2 chooses `{reference, mode 4}`, and
formations 3 and 4 choose `{reference, mode 2, mode 4}`.  This yields 72 new
terminal candidates.

Two controls are added.  The all-reference sequence fixes the target origin.
The saved v21 closest sequence
`[1,1,3,1] -> [1,1,1,4] -> [1,4,4,2]` must reproduce its registered targets
within `5e-6` percentage points.  The complete v21 terminal grid is not
rerun.  The official v24 screen therefore contains 74 sequences.

## Decision gate

- all four v23 two-step prefixes followed by reference must reproduce their
  registered six-target vectors;
- both controls and every new sequence must pass the physical, payload,
  exact-execution, rolling-B3, truth-use, repair, emergency, and
  infeasibility checks;
- strict feasibility requires all six signed targets to be nonnegative;
- strong headroom requires strict feasibility and at least `3%` mean gain.

If a strong sequence exists, it authorizes collection of broader opened-state
teacher labels and a multi-head graph-value prototype with a hard safety
projection.  It does not authorize final-model or validation claims.  If no
strong sequence exists, expansion of the present H=3 mode family stops; the
action representation or horizon must be redesigned before learning.

Seeds 223/227, X36, and final seeds remain unopened.
