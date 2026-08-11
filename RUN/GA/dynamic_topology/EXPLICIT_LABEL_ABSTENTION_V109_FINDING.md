# V109 finding: V105 already behaves as source abstention

## Paired X36 result

V109 preserves the matched static fixed-counter-clockwise route, the V105
formation schedule, control-synopsis accounting, delivery uniforms,
measurements and filter RNG. Its only intended change is to mark a delivered
selective empty payload as explicit abstention before fusion.

| Arm | Mean E-OSPA | Gain vs static | Byte saving |
|:--|--:|--:|--:|
| Static full payload | 84.037151 | -- | -- |
| V105 control-only | 79.617863 | +5.259% | +6.117% |
| V109 explicit abstention | 79.617863 | +5.259% | +6.117% |

V109 also reproduces V105's formation gains
`[-0.9312, 4.805, 7.711, 8.970, 11.250, -0.0212]%`, its F6 non-gateway
terminal loss of `-2.940%`, and its consensus changes. The registered stability
gate therefore still fails.

## Corrected causal interpretation

The earlier hypothesis that V105's empty payload participates as bulk
FoV-censored negative evidence is false for the exercised runtime path.
`collectCurrentFusionInputs` only admits a current message when the stored
payload is nonempty. V105 stores an empty payload, so the source is omitted
before `fuseLmbPosteriorsByLabel` can apply missing-label semantics. The empty
received cache also cannot produce a stale fallback. V109 makes this
abstention explicit but does not change the actual fusion inputs, which explains
the identical result.

V105's aggregate gain is therefore genuine evidence that withholding selected
cross-formation KLA inputs can help X36 under the frozen schedule. It is not
evidence that a deployable dynamic-topology policy has been found: the schedule
was selected from opened development outcomes, the whole-source action harms
F1 and F6, and the strict stability gate fails.

## Method consequence

The next action space must be label-wise rather than formation-wide. For each
receiver, source and label, the fusion layer must distinguish:

1. positive density participation;
2. credible negative-existence participation when observability supports it;
3. abstention with zero label-wise KLA weight.

Spatial weights should be normalized only across positive-density sources;
existence weights should be normalized across positive and credible-negative
sources; abstaining sources must enter neither normalization. The carrier
topology can remain connected and statically accountable while these label-wise
actions control task information flow. V108 shows that a top-three,
one-round, truth-assisted positive-label exception does not provide sufficient
headroom, so the next upper-bound experiment must score multi-step downstream
risk or use a broader label action set before training a GNN.
