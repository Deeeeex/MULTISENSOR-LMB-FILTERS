# V61 receiver-label headroom result

## Result

The first receiver--sender--label action family fails the M24 positive-control
gate at `m24-formation-fov / seed 211 / t=104`.

| Action | H=3 tracking gain | Window consensus | Attempted-byte saving | Strict |
|:--|--:|--:|--:|:--:|
| `existence-top-2` | `+0.334%` | `+0.864%` | `-0.602%` | no |
| `existence-top-4` | `+0.517%` | `+1.249%` | `-0.858%` | no |
| `existence-top-10` | `+0.517%` | `+1.248%` | `-0.569%` | no |
| `spatial-top-2` | `+0.331%` | `+0.870%` | `-0.673%` | no |
| `spatial-top-4` | `+0.331%` | `+0.870%` | `-0.529%` | no |
| `spatial-top-7` | `+0.331%` | `+0.869%` | `-0.414%` | no |

The best strict action is the full-payload reference, so the `5%` headroom
gate does not pass.

## Causal diagnosis

The intervention-step byte prediction is exact rather than miscalibrated.  For
`existence-top-4`, the label payload saves `6,688` bytes at `t=104`, exactly
matching the action-bank prediction.  Recursive posterior growth then adds
`10,152` bytes at `t=105` and `36,960` bytes at `t=106`, reversing the window
saving.  Tracking improves `1.589%` at the intervention step but changes only
`+0.013%` and `-0.032%` over the next two steps.

This reproduces the known limitation of a one-step formation action at the
same state.  The historical `+10.394%` result came from the adaptive schedule
`[1,2,4] -> [1,2] -> [4]`, not from one isolated omission.  Label granularity
alone is therefore insufficient; temporal formation control and per-label
information selection must be layered rather than substituted for one
another.

## Decision

V62 will keep the registered physical carrier graph, use the observable
formation controller to specify where and when inputs are restricted, and
allow only currently supported target information to cross those restricted
inputs.  It must account for recursive H=3 bytes and retain a full-reference
fallback.  V61 opens no X36, model-training, held-out, or validation claim.
