# V281: do not target the final output threshold first

The first bounded reference replay is complete. This is a mechanism readout
of M24 seed 1301 at three saved anchors, not a new policy or full-episode gain.

## Decision-relevant result

| Anchor | Local posterior MAP readout | Actual input label union | Existence mass before spatial overlap | After overlap | Final MAP output |
|--:|--:|--:|--:|--:|--:|
| 70 | 5.958 | 15.500 | 6.042 | 5.923 | 6.000 |
| 84 | 5.917 | 15.917 | 6.001 | 5.776 | 5.833 |
| 151 | 6.542 | 15.417 | 6.592 | 6.493 | 6.667 |

Values are means over 24 receivers per anchor. Labels are identifiers, not
truth-matched targets. The local MAP column applies the existing readout to
the cached local posterior; it is not an extra output produced in the old run.
Before-overlap mass is an algebraic decomposition with the same received
inputs and active existence weights, not a proposed fusion replacement.

- There were four lost incoming packets and no delivered empty inputs across
  72 receiver snapshots. None removed the last scheduled occurrence of a
  label from a receiver's input union. They could still remove its strongest
  existence evidence or spatial information; zero lost identifiers is not
  zero transport harm.
- Of 1,124 fused label records, 659 already had negative weighted input log
  odds before the spatial term. In 29 of those records, at least one actual
  input had existence at least 0.9. None of those 659 used a censored-absence
  participant. This distinguishes weak accumulated inputs from a smaller
  current input-weighting effect, without attributing the entire episode.
- Spatial overlap removed mean existence mass 0.119, 0.225 and 0.099, or
  1.97%, 3.75% and 1.51% of the before-overlap mass. Existence pruning removed
  only 0.00238 per receiver snapshot on average. The remaining unselected
  labels stay in the posterior; they are not discarded by MAP readout.
- The guard against an impossible powered-GM normalizer triggered on 231
  records; 92 used the powered-GM spatial result. The other records must not
  all be called failed mixture fusions: the multi-component branch need not
  be requested. This is not an exact-KLA approximation-error measurement.

## Method consequence

Do not launch an eta-floor sweep or lower the output threshold on the basis
of the current count deficit. In these snapshots the weak local estimate
exists before the final fusion, and the immediate spatial/pruning losses
are small in existence mass. Earlier repeated spatial loss, local
measurement/association updates and slow propagation remain possible causes.
The 29 strong-input/weak-pooled records retain current existence weighting
as a separate, narrower mechanism; they do not identify a winning policy.

The next useful trace should connect the pre-local-update posterior, the
updated local posterior and actual per-label received evidence across time.
An X36 trace is needed before making this a scale-general diagnosis. Any new
policy still requires paired fixed/sparse baselines and complete-set metrics.
No follow-up filter, new parameter or learned selector was started here.

## Reproduction and scope

```sh
octave --no-gui --quiet --eval "addpath(genpath(pwd)); analyzeExistenceLossLocalizationV281();"
```

The cached state is explicitly pre-topology, pre-fusion and post-local-update.
Collection mirrors the current default receiver: self first, sender-index
order, full payload threshold 0.01, absent/empty packets omitted, remaining
weights normalized, unchanged FoV-censored per-label fusion. Output counts
use existence pruning and `lmbMapCardinalityEstimate`, not an r>0.5 shortcut.
The fusion, MAP readout and reference assignment implementations have no
diff between the cache's generation commit and this replay's runtime commit.
This is a limited correspondence check, not independent verification.

The detailed record is in
`evidence/tracking_aligned_v281/m24_existence_loss_seed1301/`.
The probability-form existence identity residual is at most 2.22e-16.
The producing agent performed these checks; no independent validation,
truth-matched recall or causal decomposition of the full episode is claimed.
