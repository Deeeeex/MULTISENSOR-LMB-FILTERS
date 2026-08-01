# M24 projected-joint H=3 probe audit

## Decision

Restoring the exact joint action selected by the current truth-free
posterior-risk projector does not add H=3 deployment headroom.  Across four
already-opened seed-211 states, the strict gains are `[0, 0, 0, 0]%`.

## Provenance

- Probe generation commit: `f6268a640ac0c34d63820c015ea76ca5735de490`
- Cache generation commit: `c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53`
- Preset / seed: `m24-formation-fov / 211`
- Times: `60`, `72`, `104`, `124`
- Return: projected joint action for one step, then reference for two steps
- Feasibility: all six tracking, tail, consensus, and byte targets nonnegative
- Evidence split: outcome-inspected training mechanism probe only

| Time | Selected modes | Trust weights | Candidate six targets | Strict result |
|--:|:--|:--|:--|:--|
| 60 | `[1 1 1 1]` | `[0.7 0.7 0.7 0.7]` | `[0 0 0 0 0 0]` | reference |
| 72 | `[1 1 1 1]` | `[0.7 0.7 0.7 0.7]` | `[0 0 0 0 0 0]` | reference |
| 104 | `[3 1 1 1]` | `[0.5 0.7 0.7 0.7]` | `[-1.627 -5.718 -1.614 -2.835 +0.737 +0.758]` | reference |
| 124 | `[1 1 1 1]` | `[0.7 0.7 0.7 0.7]` | `[0 0 0 0 0 0]` | reference |

At times 60 and 72, the exact projector collapses to reference even though the
larger singleton/pair teacher bank contains strict gains of `+1.591%` and
`+0.024%`.  At time 104, it chooses formation 1 at trust 0.50 although that
action is known to hurt the H=3 return.  The current one-round posterior-risk
objective therefore fails both by omission and by misranking.

## Consequence

This probe falsifies a narrow repair: keeping the full mode vector already
chosen by the exact projector.  It does not falsify coordinated dynamic
topology in general.  The next mechanism question is intervention duration.
On a 24-node recursive filter, changing one routing step may be too weak to
produce the required gain, while blindly persisting can accumulate tail or
consensus damage.  A duration probe on the same opened states will compare the
known one-step oracle action with a three-step frozen macro-action under the
same six-target and topology runtime gates.

Seeds 223, 227, X36, and all final seeds remain unopened by this audit.
