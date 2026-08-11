# V106 finding: reactive release is too late and creates a switch shock

## Matched result

| Arm | Mean gain | Min. formation | F6 peer terminal | Byte saving |
|:--|--:|--:|--:|--:|
| V105 persistent protection | +5.259% | -0.931% | -2.940% | +6.117% |
| V106 same-page release oracle | +5.246% | -0.865% | -2.947% | +5.286% |

V106 restores F1 to the static full payload at t=77 and F6 at t=78, the
pages where their opened V105 formation or peer outcomes first reverse sign.
Every topology and fusion-weight row remains static, the frozen reference is
reused, and the candidate still improves network E-OSPA by 5.246% while
saving 5.286% attempted bytes.  The network mean headroom therefore survives
release.

## Why the local gate still fails

Reactive release does not undo the state already accumulated during earlier
protected pages.  It also creates an immediate discontinuity when the full
cross-formation payload returns:

- F1 at t=77 worsens from -5.861% under V105 to -12.181% under V106.  Its
  terminal loss improves from -15.754% to -5.681%, so recovery begins only
  after the initial switch shock.
- F6 at t=78 remains approximately neutral, then falls to -2.406% at t=79;
  its five non-gateway peers remain at -2.947% terminal gain.
- F2 still has a one-page -4.928% transient at t=76, showing that the
  formation response is not captured by a single global dwell limit.

The strict gate fails even though the network mean, consensus and
communication metrics remain positive.  A controller that waits for a
current outcome or current-state sign reversal cannot be safe: by the time
the reversal is visible, the pre-fusion posterior has already moved into a
harmful region.

## Next decision

V107 advances the two release pages by one step: F1 returns at t=76 and F6 at
t=77.  This is still a retrospective headroom oracle, but it distinguishes a
late reaction from a fundamental failure of coarse formation-level temporal
control.  No threshold search is needed.

- If early release makes F1/F6 nonnegative while retaining at least 5% mean
  gain, a causal controller should predict one-step-ahead protection debt.
- If it merely shifts the switch shock earlier, binary full/control-only
  release is structurally inadequate and the method must use a gradual or
  receiver-selective reintroduction of cross-formation information.

