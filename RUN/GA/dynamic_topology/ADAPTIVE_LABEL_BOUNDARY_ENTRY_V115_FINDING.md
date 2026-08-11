# V115 finding: local label confidence cannot predict delayed value

## Matched result

| Boundary rule | Mean E-OSPA | Gain vs CW | vs V113 | vs V114 | Mature min | Min formation | F6 peers | Bytes | Gate |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|:--:|
| sender support | 78.479671 | +4.063% | +0.000% | -0.205% | +2.588% | -1.495% | -7.206% | +3.419% | fail |
| support or high existence | 78.479683 | +4.063% | +0.000% | -0.205% | +2.588% | -1.495% | -7.206% | +3.373% | fail |
| receiver need | 78.479672 | +4.063% | +0.000% | -0.205% | +2.588% | -1.495% | -7.206% | +3.359% | fail |

V113 unshielded is `78.479689`; V114 early whole-posterior shielding is
`78.319230`.  All candidates preserve rolling B3.

## What the screen isolates

V115 does not change the V113 F2--F5 mechanism or the clockwise physical
carrier.  It changes only F6 boundary participation.  Admitted labels carry
their complete Bernoulli Gaussian mixtures.  Omitted labels explicitly
abstain from that label's KLA update, so the result is not caused by
censored-missing-label semantics.

The three rules admit different numbers of labels on pages 5--8:

| Rule | Selected labels on pages 5--8 |
|:--|:--|
| sender support | `[19 16 18 19]` |
| support or high existence | `[19 19 19 19]` |
| receiver need | `[21 19 20 22]` |

Despite these differences, all tracking and F6-tail outcomes match V113.
The additional labels admitted by the more permissive rules change bytes but
carry negligible net task value.  More importantly, the supported labels
already reproduce the complete delayed F6 loss.  A label can look locally
supported or necessary and still have negative recursive value after later
fusion and extraction.

## Method consequence

Do not tune support, existence or receiver-need thresholds and do not train a
GNN against these one-step proxy labels.  The next bounded experiment is a
privileged H=8 label-return oracle on `27 -> 32`:

1. use complete-label keep/drop decisions with explicit abstention;
2. evaluate delayed network and F6 return, not immediate receiver risk;
3. form a small Pareto frontier over mean gain, F6 tail and bytes; and
4. retain V113 full entry and V114 empty entry as the two endpoints.

If an intermediate subset passes, the learning target becomes delayed
label-wise residual value on the time-expanded influence graph.  If none
passes, label-wise boundary control is not the missing action and the next
upper bound must change the physical source or carrier.

V115 is opened X36 seed-211 t=72 development evidence.  It is not validation
or cross-scene generalization.
