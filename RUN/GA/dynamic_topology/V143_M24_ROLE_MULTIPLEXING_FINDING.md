# V143 M24 finding: budget feasibility is solved, temporal headroom is not

V143 fails its frozen M24 mechanism gate because intervention-window E-OSPA
gain is only `+1.882%`, below the preregistered `+5%` threshold.  The result
is therefore a repository-only experiment record: X36 is not opened, and no
number from this screen is copied into the main progress document.

## Frozen result

- Source commit: `5ac98f1`.
- Preset / seed / anchor / action: `m24-formation-fov` / `1601` / `95` / `25`.
- Intervention gain: `+1.882%`.
- Full-window / mature-window gain: `+5.554% / +6.159%`.
- Minimum sensor / formation gain: `+0.790% / +3.354%`.
- Whole-formation W-to-multiplexed-R rejoin match: `100.000%`.
- Attempted-byte delta: `-0.722%`.
- Reference / working wire roles: `2048 / 72`, exactly one role on each of
  the `2120` attempted messages.
- Auxiliary attempted bytes: exactly zero.
- Reference / candidate runtime: `619.13 / 1119.94 s`; the current dual-state
  implementation is about `80.9%` slower and is not yet deployment-ready.

## Temporal attribution

The 72 working-role messages occur only on continuation pages 2, 4 and 6,
with counts `36`, `27` and `9` as formations rejoin.  Mean page gains over the
six intervention pages are

```text
[+3.635, +10.595, +7.012, +5.671, -8.398, -8.067]%.
```

The role split is therefore not too weak everywhere.  It produces a strong
positive response after the first working page, but the later reference/rejoin
transition creates a short negative block.  From page 9 onward the candidate
is mostly positive, reaches about `+20.1%` at page 23, and retains a
`+6.159%` mature-window improvement.  This explains the unusual combination
of a failed short intervention gate and strong, uniformly positive
full-window sensor and formation means.

The `100%` rejoin statistic must not be described as equality to the
independently executed static reference.  W rejoins the causal multiplexed R
lineage.  R itself omits intra-formation inputs on working-role pages and can
therefore differ from the static carrier; that controlled state-flow change
is also what produces the long-tail tracking effect.

## Method decision

V143 establishes the missing communication fact: a mixture-aware W/R method
can use exactly one posterior per attempted edge, add no auxiliary payload,
reduce rather than increase attempted bytes, and avoid every average sensor
or formation regression.  It does not yet establish the required immediate
tracking gain.

Before this outcome opened, V144 froze a single successor schedule from the
nominal mixing matrices alone: `R-W-W-W`.  On M24 this changes only page 3
from R to W relative to V143, preserving the page-5 reference anchor while
increasing useful local propagation.  No alternative schedule or threshold
may be selected from this outcome.  V144 is run once on M24; failure closes
fixed periodic role scheduling and moves the method to state-dependent edge
roles rather than another cadence sweep.

