# V136 finding: local propagation restores headroom but remains unsafe

V136 is a repository-only mechanism result.  It fails the registered M24
sensor gate, so X36 was not run and the result must not be copied into the
main progress document.

## Frozen M24 result

- All ranked formations use staggered binary reentry.
- Intervention E-OSPA gain: **+5.800%**, above the +5% mechanism target.
- Full-window / mature-window gain: **+0.528% / +0.000%**.
- Minimum formation gain: **+0.000%**; minimum sensor gain:
  **-0.681%** at sensor 11 in formation 2.
- Whole-formation reentry matches the reference relay exactly on every
  unprotected page: **100%**.
- The charged compound `W + R` representation increases attempted bytes by
  **89.182%**.  V136 is therefore not a communication-saving method.

## Mechanism interpretation

Allowing W to propagate inside a formation raises the V135 intervention gain
from `+3.649%` to `+5.800%`.  Per-page network gains reach
`[+3.635, +7.336, +13.105, +5.779, +0.532, +4.641]%` before exact reentry.
This establishes that the useful local protection signal can produce material
network-level tracking headroom when it is allowed to spread spatially.

The remaining failure is localized rather than common-mode.  In formation 2,
sensors 7--8 benefit while sensors 9--12 regress after receiving the changed
within-formation state; page-five losses reach about `-40.7%` at sensors 11
and 12.  In contrast, formation 4 propagates the same kind of working state
with uniformly positive results.  A formation-wide W broadcast is therefore
still too coarse.

## Method decision

The next bounded action should keep the V136 payload and state semantics but
add a causal node-local safety choice.  At each protected node, compare W and
R against that node's current measurement evidence and retain W only when it
has the better proper predictive score; otherwise use R immediately.  The
decision must use no target truth or future outcome, add no payload beyond the
already charged V136 compound message, and preserve exact whole-formation
reentry.  Failure closes this dual-state branch before X36 or compression;
an M24 pass authorizes the paired X36 mechanism screen.
