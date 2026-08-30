# V161 observable one-hop label-value gate

- Preset / seed: `x36-formation-fov / 211`
- Registered receiver-time cells: `36`
- Frozen source rule: `minimum-current-per-label-posterior-bayes-risk`
- Same-source-set positive oracle: `144` actions / `1036.323678` gain
- Conservative risk/rich synopsis bytes: `250456 / 1862528`

- Nominal base-route byte headroom: `1679568`

| Observable label policy | Selected | Positive / negative | Strong positive | Truth gain | Oracle capture | Harmful cells | Minimum cell gain | Synopsis | Control total B | Adjusted byte saving |
|:--|--:|:--|--:|--:|--:|--:|--:|:--|--:|--:|
| risk-reduction-k4 | 144 | 108 / 7 | 104 | 687.236923 | 66.315% | 0 | 12.104559 | risk | 727000 | +3.333% |
| risk-reduction-k2 | 72 | 60 / 1 | 57 | 362.008961 | 34.932% | 1 | -0.083428 | risk | 494104 | +4.148% |
| confidence-disagreement-k4 | 144 | 134 / 10 | 132 | 923.359048 | 89.099% | 0 | 17.903203 | rich | 2345792 | -2.331% |
| handover-rescue-k4 | 144 | 142 / 2 | 139 | 1001.379236 | 96.628% | 0 | 14.911085 | rich | 2345792 | -2.331% |
| handover-rescue-k2 | 72 | 71 / 1 | 71 | 458.174323 | 44.212% | 0 | 5.793536 | rich | 2104160 | -1.486% |
| risk-gated-handover-k4 | 144 | 142 / 2 | 139 | 1001.379236 | 96.628% | 0 | 14.911085 | rich | 2345792 | -2.331% |

- Safety-first development policy: `risk-reduction-k4`

| t | F | Receiver | Oracle gain/actions | Selected-policy gain/actions |
|--:|--:|--:|:--|:--|
| 76 | 2 | 7 | 27.0700/4 | 12.1046/4 |
| 76 | 2 | 8 | 31.9057/4 | 17.6524/4 |
| 76 | 2 | 9 | 26.9715/4 | 12.2004/4 |
| 76 | 2 | 10 | 31.6630/4 | 17.6557/4 |
| 76 | 2 | 11 | 27.0921/4 | 12.2075/4 |
| 76 | 2 | 12 | 26.5605/4 | 12.1408/4 |
| 77 | 1 | 1 | 27.3429/4 | 19.2795/4 |
| 77 | 1 | 2 | 27.4549/4 | 19.2327/4 |
| 77 | 1 | 3 | 27.4868/4 | 26.7344/4 |
| 77 | 1 | 4 | 27.5627/4 | 26.7813/4 |
| 77 | 1 | 5 | 27.6054/4 | 19.1119/4 |
| 77 | 1 | 6 | 27.4891/4 | 19.3131/4 |
| 78 | 1 | 1 | 35.1508/4 | 23.8526/4 |
| 78 | 1 | 2 | 35.1525/4 | 14.8102/4 |
| 78 | 1 | 3 | 30.4457/4 | 21.1202/4 |
| 78 | 1 | 4 | 30.5060/4 | 21.1330/4 |
| 78 | 1 | 5 | 30.5493/4 | 21.1721/4 |
| 78 | 1 | 6 | 34.8053/4 | 15.0514/4 |
| 78 | 6 | 31 | 25.1411/4 | 17.5915/4 |
| 78 | 6 | 32 | 25.1135/4 | 17.6697/4 |
| 78 | 6 | 33 | 27.1827/4 | 19.4486/4 |
| 78 | 6 | 34 | 32.6381/4 | 24.9672/4 |
| 78 | 6 | 35 | 25.1169/4 | 24.9861/4 |
| 78 | 6 | 36 | 25.0985/4 | 17.8714/4 |
| 79 | 1 | 1 | 28.9333/4 | 16.7365/4 |
| 79 | 1 | 2 | 28.6801/4 | 16.9780/4 |
| 79 | 1 | 3 | 15.2729/4 | 14.9911/4 |
| 79 | 1 | 4 | 35.0109/4 | 23.8924/4 |
| 79 | 1 | 5 | 28.6426/4 | 16.9162/4 |
| 79 | 1 | 6 | 35.3222/4 | 15.0623/4 |
| 79 | 6 | 31 | 27.4433/4 | 19.3734/4 |
| 79 | 6 | 32 | 30.8759/4 | 21.4304/4 |
| 79 | 6 | 33 | 30.7224/4 | 21.4895/4 |
| 79 | 6 | 34 | 27.4417/4 | 27.4417/4 |
| 79 | 6 | 35 | 27.4359/4 | 19.4183/4 |
| 79 | 6 | 36 | 27.4371/4 | 19.4189/4 |

## Evidence boundary

V161 reuses the privileged 36 V157 receiver-time cells but does not reuse their selected labels. Every observable policy ranks all currently available one-hop labels with present-time posterior, evidence and FoV metadata; truth only evaluates chosen actions. The positive oracle also uses truth and is an offline mechanism upper bound. Reported gains are sums of immediate cell-wise E-OSPA marginals, not recursive tracking outcomes. Synopsis byte estimates are conservative directed-message charges but are not yet inserted into the full filter accounting.
