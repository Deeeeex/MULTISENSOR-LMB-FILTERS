# M24 complementary H=3 local-sequence audit

## Decision

The v18 local-action beam does not produce a deployable M24 sequence.  Among
47 evaluated H=3 sequences, only `[1,1,1]` is nonnegative in all six targets;
the strict oracle therefore remains reference with `0%` gain.

## Provenance

- Generation commit: `b38a4137ed48607d7fc72b2ae473fc3b79c098ab`
- Cache generation commit: `c9c6d4dcdc7ad1cb04fb88a22823e99c7fc5bc53`
- Preset / state: `m24-formation-fov / seed 211 / t=72`
- Stage-one / stage-two / unique arms: `11 / 37 / 47`
- Selected second actions: `[13,12,1,11]`
- Per-step runtime failures: none
- Strict-feasible sequences: `1/47`

The registered `[9,1,1]` arm reproduces the v17 ceiling: `+5.988%` mean
tracking and `-11.486%` consensus.  This confirms that the probe starts from
the intended high-value/high-risk mechanism rather than a changed cache or
reference.

## Best local repair trajectory

| Sequence | Mean | Min. formation | Worst sensor | Consensus | Attempted bytes | Delivered bytes | Strict |
|:--|--:|--:|--:|--:|--:|--:|:--:|
| `[9,1,1]` | +5.988% | 0.000% | -0.001% | -11.486% | +0.167% | +0.175% | no |
| `[9,13,1]` | +7.911% | 0.000% | -0.001% | -6.069% | -0.844% | -0.883% | no |
| `[9,13,11]` | +8.624% | 0.000% | -0.001% | -3.739% | +0.023% | +0.024% | no |
| `[9,13,12]` | +8.623% | 0.000% | -0.001% | -3.717% | +0.023% | +0.024% | no |
| reference | 0.000% | 0.000% | 0.000% | 0.000% | 0.000% | 0.000% | yes |

The path `[9,13,12]` repays approximately `67.6%` of the original consensus
debt while increasing tracking gain and restoring both byte targets.  The
remaining `-3.717%` consensus target—not communication or formation-tail
quality—is the material blocker.  The `-0.001%` worst-sensor value is also
formally negative, but it is orders of magnitude smaller and does not change
the mechanism diagnosis.

## Consequence

The result rejects a local-action-only H=3 controller, not temporal control in
general.  The monotone debt reduction from `[9,1,1]` to `[9,13,1]` and then
`[9,13,12]` is evidence that action order matters and that repair is possible,
but a single formation per step cannot synchronize the other formations fast
enough.  The next minimal action-space expansion is to retain the fixed
`[9,13]` prefix and enumerate all six conservative two-formation actions at
step three under the same strict targets and runtime gates.

This is a privileged, single-opened-state mechanism probe.  It does not train
or validate a selector, and seeds 223/227, X36, and all final seeds remain
unopened.
