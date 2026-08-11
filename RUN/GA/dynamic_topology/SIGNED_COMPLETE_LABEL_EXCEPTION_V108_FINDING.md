# V108 finding: sparse positive labels do not repair censored-absence harm

## Matched result

| Arm | Mean gain | F1 | F6 | F6 peer terminal | Byte saving |
|:--|--:|--:|--:|--:|--:|
| V105 control-only | +5.259% | -0.931% | -0.021% | -2.940% | +6.117% |
| V108 signed labels | +5.237% | -0.930% | -0.151% | -2.904% | +5.870% |

V108 admits 24 complete Bernoulli Gaussian-mixture labels on actually
delivered F1/F6 gateway pages.  Every label has positive one-round capped
expected-risk value under the opened V105 posterior and target truth, and no
selected action causes a downward `r=0.5` crossing in that one-round
counterfactual.  The recursive tracking result nevertheless fails the gate.
F1 is numerically unchanged, F6 becomes 0.129 percentage points worse over the
window, and the F6 peer terminal loss improves by only 0.036 points.  The
additional payload reduces byte saving by 0.247 points without improving the
network mean.

## Why the oracle signal does not transfer

The signed score evaluates one receiver-label immediately after one fusion
round.  The tracking outcome depends on later MAP cardinality extraction,
mixture-component selection and propagation through the formation.  A
positive local density-risk change can therefore be diluted or reversed after
recursive fusion.  More fundamentally, the V105 fallback is not a neutral
absence of information.

On a selective edge, the runtime transmits an empty heavy payload plus the
control synopsis and still inserts that empty payload as a fusion source.  In
`fov-aware-censored` mode, a label missing from that source is assigned the
configured low-existence bound whenever the source sensor is judged able to
observe the label.  Thus the current control-only action can supply negative
existence evidence for every omitted label.  V108 changes 24 labels from that
negative/empty state to full positive densities, but leaves the bulk
censored-absence mechanism unchanged.  Its failure does not show that
label-wise control is useless; it shows that positive-label restoration alone
cannot repair a carrier whose default state conflates abstention and negative
evidence.

## Method decision

Do not train a GNN from the V108 oracle labels and do not search another top-k
or risk threshold.  The next causal experiment is explicit abstention
semantics on the same static route and V105 formation schedule:

1. the synopsis and its communication cost remain;
2. an abstaining cross source has zero weight for every omitted label and is
   not interpreted as a low-existence observation;
3. complete positive labels and credible negative evidence become separate
   future actions; and
4. the matched static full-payload baseline, delivery uniforms and H=8 window
   remain frozen.

This V109 attribution will determine whether V105's mean gain comes primarily
from useful source abstention or from mass censored-absence injection.  Only
after that separation should the method optimize a ternary per-label action:
positive density, credible negative evidence, or abstention.
