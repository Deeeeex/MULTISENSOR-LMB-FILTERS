# V284: interpretation of the matched-RMSE increase

Post-hoc, self-check only. The original joint screen remains failed; these additional quantities do not replace its RMSE definition.

Each arm uses minimum-sum-distance assignment to the same truth positions at the same sensor-time. Common targets are the intersection of assigned truth indices, not confirmed track-label identities or an independent evaluation set.

The existing vectorized `munkres` solver is used for speed; any per-cell RMSE mismatch invokes the original `Hungarian` solver. Fallbacks: 0. Maximum difference from the saved official per-cell RMSE: 1.42e-14 m.

| Scope | Common pairs | Added pairs | Lost pairs | Common reference pooled RMSE | Common candidate pooled RMSE | Added-target pooled RMSE |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| All | 5868 | 7850 | 610 | 8.730201 | 8.897792 | 34.730995 |
| Formation 1 | 1233 | 1315 | 113 | 9.268526 | 9.653859 | 19.494156 |
| Formation 2 | 1080 | 1474 | 92 | 7.820924 | 7.827251 | 13.349694 |
| Formation 3 | 886 | 1541 | 109 | 8.640125 | 8.631307 | 16.637272 |
| Formation 4 | 927 | 1156 | 49 | 9.335967 | 9.642784 | 32.819468 |
| Formation 5 | 832 | 1166 | 105 | 8.745034 | 8.874302 | 50.215719 |
| Formation 6 | 910 | 1198 | 142 | 8.431323 | 8.501447 | 58.463069 |

Pooled RMSE is sqrt(total squared error / matched-pair count), not the official average of per-cell RMSE. Pair counts repeat targets across sensors/time and are not independent samples. Added/lost mean assignment-set membership relative to the reference, not a causal identity ground truth.

## Cached fixed-routing prefix (original fusion rule)

| Metric | Fixed routing | V284 on sparse routing |
| --- | ---: | ---: |
| E-OSPA | 134.851325 | 117.117031 |
| Official conditional RMSE | 8.753521 | 22.303075 |
| Count error | 19.405556 | 14.473611 |
| Entire-prefix disagreement | 144.381812 | 115.117106 |

The cached fixed summary has no per-step payload-byte array; its 160-step bytes must not be compared with V284 40-step bytes. A fixed-routing arm with V284 semantics has not run. No joint improvement or routing attribution is established by this extra readout.
