# M24 H=3 risk-tolerance audit

## Decision

The exact samplewise zero-regression rule is not the sole reason the v13 M24
oracle is weak.  Small auxiliary tolerances up to `0.5%` select exactly the
same actions as the strict gate.  Even at `2%`, the 19-action bank provides no
state with at least `3%` jointly bounded gain.

## Provenance and reproduction

- Audit generation commit: `6e70e27702994604b2bd374ed4a4684969fb344e`
- Frozen target generation commit: `c32a0649834e0119b3993a95e9e441e6269c8c3b`
- Preset / states: `m24-formation-fov`, seeds `[211,223,227]`, times `[60,72]`
- Actions: registered reference plus 12 singleton and six conservative-pair
  interventions
- Tolerance grid: `[0,0.1,0.25,0.5,1,2]` percentage points
- Strict reproduction: `[1.590662,0.024472,0,0.516168,0,0.751288]%`

The script stops unless its `tau=0` row reproduces the prior strict audit to
within `5e-6` percentage points.  It reselects only from stored target
matrices; no filter state, action, measurement, or seed is regenerated.

| Diagnostic selector | Tau | Positive states | Strong states | Mean oracle gain | Communication-regression states | Gate |
|:--|--:|--:|--:|--:|--:|:--|
| Estimation auxiliary tolerance, bytes strict | 0.0 | 4/6 | 0/6 | +0.480% | 0/6 | FAIL |
| Estimation auxiliary tolerance, bytes strict | 0.5 | 4/6 | 0/6 | +0.480% | 0/6 | FAIL |
| Estimation auxiliary tolerance, bytes strict | 1.0 | 4/6 | 0/6 | +0.480% | 0/6 | FAIL |
| Estimation auxiliary tolerance, bytes strict | 2.0 | 5/6 | 0/6 | +0.928% | 0/6 | FAIL |
| All auxiliary targets tolerant | 0.5 | 4/6 | 0/6 | +0.480% | 0/6 | FAIL |
| All auxiliary targets tolerant | 1.0 | 6/6 | 0/6 | +1.778% | 4/6 | FAIL |
| All auxiliary targets tolerant | 2.0 | 6/6 | 0/6 | +1.857% | 3/6 | FAIL |

## What the unconstrained ceiling shows

If all five auxiliary targets are ignored after the outcomes are known, the
mean-only oracle gains are
`[2.323,5.988,1.973,0.516,2.891,0.751]%`: six positive states, one strong
state, and `+2.407%` mean.  This is not a candidate policy.  It separates two
facts that a single aggregate number would conflate:

- the bank contains actions capable of changing network-mean tracking;
- its largest tracking gain is obtained by accepting severe network
  disagreement.

At seed 211/time 72, `formation-3-dynamic-trust-0.50` produces `+5.988%`
network-mean gain but `-11.486%` consensus gain.  At seed 227/time 60, the
mean-best pair obtains `+2.891%` while losing `-1.829%` on the minimum
formation and `-3.156%` on consensus.  Treating these actions as safe by
loosening a scalar threshold would change the research objective rather than
solve it.

## Consequence

The next method should expand the temporal action space, not merely the risk
tolerance.  A useful falsifier is an H=3 sequence bank in which the first
nonreference action may exploit a current information advantage and later
nonreference actions are chosen to restore formation and consensus quality.
The original terminal six-target non-regression rule and every per-step
physical/B3/runtime check remain unchanged.  A privileged exhaustive sequence
screen on the already-opened states must show strong safe headroom before any
GNN or causal selector is trained.

This audit is outcome-inspected mechanism evidence only.  It does not
authorize a relaxed deployment rule or any M24/X36 performance claim, and it
opens no new seeds.
