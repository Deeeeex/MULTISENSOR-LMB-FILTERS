# V292: path existence versus weighted observation transport

Opened M24/X36 seed 1301, original 160-step scenes. Source `ae81dcc2775dbd07ec5792c91a7637c1481bd1de`. Self-check only. No filter, policy or parameter changes.

For each active sensor-target-time triple and window t-h:t, initialize z=0 and iterate z_k = W_k [o_k + (1-o_k).*z_(k-1)]. W is the delivered packet-level row-stochastic matrix; o is the cached geometric visibility indicator. A current opportunity is inserted before one synchronous fusion round.

z is the probability that a backwards stochastic source path, sampled using these weights, encounters an observation opportunity in that window. It is not posterior mass, label recall, detection probability, or a tracking-error bound. Delivered empty packets, label-specific censoring, spatial overlap, local likelihoods, pruning and association are not represented.

| Scale | Arm | Maximum age | Path coverage | Mean weighted source-hit score | Mean / median / p10 score, given a path |
| --- | --- | ---: | ---: | ---: | --- |
| N=24 | Fixed tree | 0 | 31.133% | 25.192% | 0.809167 / 1.000000 / 0.050000 |
| N=24 | Fixed tree | 3 | 52.607% | 28.898% | 0.549306 / 0.891094 / 0.003234 |
| N=24 | Fixed tree | 8 | 66.777% | 31.807% | 0.476319 / 0.095476 / 0.001220 |
| N=24 | Fixed tree | 16 | 72.886% | 34.831% | 0.477887 / 0.138828 / 0.003116 |
| N=24 | Full causal repair | 0 | 32.214% | 25.193% | 0.782065 / 1.000000 / 0.050000 |
| N=24 | Full causal repair | 3 | 59.852% | 29.041% | 0.485207 / 0.241519 / 0.002500 |
| N=24 | Full causal repair | 8 | 87.337% | 32.142% | 0.368020 / 0.062696 / 0.000225 |
| N=24 | Full causal repair | 16 | 95.939% | 35.451% | 0.369512 / 0.101171 / 0.001444 |
| N=24 | Sparse causal repair | 0 | 31.133% | 25.174% | 0.808614 / 1.000000 / 0.050000 |
| N=24 | Sparse causal repair | 3 | 50.153% | 28.995% | 0.578125 / 0.950658 / 0.017270 |
| N=24 | Sparse causal repair | 8 | 81.261% | 32.129% | 0.395381 / 0.066587 / 0.000966 |
| N=24 | Sparse causal repair | 16 | 95.308% | 35.397% | 0.371396 / 0.100213 / 0.001594 |
| N=36 | Fixed tree | 0 | 20.593% | 16.163% | 0.784874 / 1.000000 / 0.050000 |
| N=36 | Fixed tree | 3 | 36.034% | 19.129% | 0.530868 / 0.710500 / 0.003160 |
| N=36 | Fixed tree | 8 | 49.716% | 21.459% | 0.431627 / 0.078757 / 0.000672 |
| N=36 | Fixed tree | 16 | 62.573% | 23.867% | 0.381426 / 0.102038 / 0.000087 |
| N=36 | Full causal repair | 0 | 21.662% | 16.167% | 0.746352 / 1.000000 / 0.050000 |
| N=36 | Full causal repair | 3 | 43.095% | 19.267% | 0.447069 / 0.069469 / 0.001939 |
| N=36 | Full causal repair | 8 | 68.388% | 21.805% | 0.318839 / 0.055437 / 0.000216 |
| N=36 | Full causal repair | 16 | 91.440% | 24.527% | 0.268233 / 0.052908 / 0.000011 |
| N=36 | Sparse causal repair | 0 | 20.831% | 16.152% | 0.775360 / 1.000000 / 0.050000 |
| N=36 | Sparse causal repair | 3 | 35.688% | 19.237% | 0.539046 / 0.784000 / 0.017150 |
| N=36 | Sparse causal repair | 8 | 63.183% | 21.806% | 0.345128 / 0.060862 / 0.000833 |
| N=36 | Sparse causal repair | 16 | 88.518% | 24.496% | 0.276735 / 0.060880 / 0.000043 |

All four pre-existing horizons 0, 3, 8 and 16 are reported, with shorter windows at episode start. Percentages average over active sensor-target-time triples, not independent trials; no confidence intervals or significance claim. Positive weighted score exactly matches the cached V280 binary reachability at each horizon; route message counts also match.

This diagnostic tests whether binary temporal paths overstate weighted access. It does not establish that stronger mixing improves tracking; earlier stronger-mixing controls were unfavorable, and changing weights would also alter density compatibility. No automatic weight sweep or paper-method promotion follows.
