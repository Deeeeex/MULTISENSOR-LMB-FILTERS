# V118 finding: source identity is a weak signal, not a sufficient control

## Result

V118 replaces the `27 -> 32` F5-to-F6 sender with each other F5 sensor.  A
literal one-edge replacement would strand sensor 27, so every arm applies the
same minimum connectivity repair, `25 -> 26` to `27 -> 26`.  All five paired
arms preserve row sums, the 60-message budget, the nonzero fusion-weight
multiset, static strong connectivity and rolling B3.  None passes the X36 H=8
tracking gate.

| F5 source to sensor 32 | Mean E-OSPA | Gain vs CW | vs V113 | vs V114 | Mature min. | Min. formation | F6 peers | Worst | Byte saving |
|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|
| 25 | 78.609175 | +3.905% | -0.165% | -0.370% | +3.038% | -2.326% | -7.412% | +6.089% | +3.213% |
| 26 | 78.598787 | +3.918% | -0.152% | -0.357% | +2.677% | -2.232% | -7.234% | +6.601% | +3.034% |
| 28 | 78.426149 | +4.129% | +0.068% | -0.137% | +3.418% | -0.991% | -7.305% | +9.616% | +3.304% |
| 29 | 78.512258 | +4.023% | -0.041% | -0.246% | +2.838% | -1.761% | -7.260% | +8.083% | +3.175% |
| 30 | 78.525158 | +4.008% | -0.058% | -0.263% | +2.675% | -2.130% | -0.416% | +6.237% | +2.908% |

Source 28 has the best network mean and worst-sensor gain; source 30 has the
smallest F6-peer regression and is therefore the minimum-gate-regret oracle.
The two orderings disagree.  More importantly, the entire mean range across
five source identities is only `0.183026` E-OSPA, and even the best mean arm
remains `0.137%` worse than the V114 empty-boundary endpoint.  The source
identity is therefore observable in the tail, but it cannot supply the missing
one percentage point of network gain or remove the negative F6 tail.

## Interpretation

The binding failure is not topology validity, communication, consensus or an
isolated worst sensor.  Every candidate improves consensus by more than 10%,
saves roughly 3% attempted bytes and improves the worst sensor.  Yet the mean
gain remains below 5%, the weakest formation remains negative, and four of
five arms make the F6 non-receiver peers about 7% worse.  Source 30 nearly
removes the peer-tail loss but still leaves F6 down 2.130% as a formation.

V116--V118 now jointly close the local single-boundary family: privileged
label choice does not help, moving the F6 receiver makes the delayed loss
worse, and changing the F5 source produces only a weak ranking signal.  A GNN
trained on label, sender or receiver identity inside this action family would
learn to choose among actions that all fail the actual tracking objective.

## Next decision

The next screen must alter formation-level information provenance while
retaining an exact message budget and the same safety projection.  It should
not repeat the one-round generic multi-source reallocation rejected by V95.
The useful hypothesis is instead horizon-aware: give the delayed F5-to-F6
boundary a second time-expanded path, paid for by removing an already shielded
cross-formation input, and compare it with both the original clockwise carrier
and the same donor deletion without the new path.  This separates genuine
complementary provenance from the benefit of suppressing a harmful input.

V118 is privileged opened-development evidence at X36 seed 211, t=72, H=8.
It is not deployable, validation, or generalization evidence.
