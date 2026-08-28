# V144 M24 finding: fixed periodic role scheduling is closed

V144 fails its frozen M24 gate.  Its intervention-window E-OSPA gain is
`+3.670%`, below the registered `+5%` threshold, while the worst sensor and
formation regress by `8.228%` and `3.727%`.  This is therefore a
repository-only experiment record: X36 is not opened and no V144 result is
copied into the main progress document.

## Frozen result

- Source commit: `25a4db9`.
- Preset / seed / anchor / action: `m24-formation-fov` / `1601` / `95` / `25`.
- Frozen cadence: repeating `R-W-W-W`, followed by all-reference operation
  after protected formations disappear.
- Intervention / full-window / mature-window gain:
  `+3.670% / +1.832% / +1.963%`.
- Minimum sensor / formation gain: `-8.228% / -3.727%`.
- Whole-formation W-to-multiplexed-R rejoin match: `100.000%`.
- Attempted-byte delta: `-1.700%`.
- Reference / working wire roles: `2012 / 108`, with one posterior and no
  auxiliary payload on each attempted edge.
- Output-only predictive fallbacks: `33`.
- Gate: fail.

## Decisive comparison with V143

Relative to V143's alternating `R-W` carrier, V144 raises the number of
working-role messages from `72` to `108`.  This increases the intervention
gain from `+1.882%` to `+3.670%`, but changes the minimum-sensor gain from
`+0.790%` to `-8.228%`.  More W traffic is therefore useful in aggregate but
harmful when assigned uniformly by page: the value of W depends on the
current edge, posterior disagreement and downstream information path, not on
a global periodic cadence.

## Method decision

The preregistered fixed-schedule branch ends here.  No additional cadence,
phase offset or W fraction will be tuned on this seed.  The next method must
select the W/R role per eligible edge from observable online state, then pass
the raw scores through a deterministic projector that enforces:

1. exactly one valid mixture-aware LMB posterior per attempted edge;
2. no W payload on cross-formation edges;
3. bounded reference age and time-expanded reference connectivity;
4. attempted bytes no greater than the static-reference carrier; and
5. all-reference fallback whenever the learned proposal is infeasible.

V142 supplies the teacher target: sparse reference-supported spatial state is
the missing information on X36.  V143 supplies the deployment constraint: the
two posterior roles can be multiplexed without extra payload.  V144 supplies
the scheduling conclusion: deciding the role requires state- and edge-aware
selection rather than another fixed timing rule.
