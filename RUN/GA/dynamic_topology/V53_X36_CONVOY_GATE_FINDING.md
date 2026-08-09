# V53 X36 convoy gate finding

## Frozen decision

V53 fails the pre-registered X36 development gate and is rejected. No M24 or
additional X36 scene runs are justified for this arm. The next tracking arm is
the receiver-safe, label-selective V54 oracle; V53 thresholds and cross-pulse
timing will not be tuned further.

## Paired result

The candidate reused the saved V46 X36 convoy reference for scene seed 1009,
delivery seed 49,101,009, and filter seed 49,601,009.

| Metric | V46 | V53 | Improvement |
|:--|--:|--:|--:|
| Full-horizon position E-OSPA | 126.370 | 127.196 | -0.65% |
| Focus-window position E-OSPA | 123.494 | 123.868 | -0.30% |
| Worst-sensor position E-OSPA | 134.295 | 133.881 | +0.31% |
| Mean absolute cardinality error | 14.534 | 14.763 | -1.58% |
| Mean inter-formation position OSPA | 120.032 | 119.766 | +0.22% |
| Attempted messages | 7,200 | 7,140 | +0.83% saving |
| Attempted posterior payload bytes | 233,938,560 | 214,527,264 | +8.30% saving |
| Runtime | 3,036.6 s | 3,900.3 s | 28.45% slower |

V53 passes only the focus and attempted-message constraints. It fails the
required `+1%` full-horizon and cardinality improvements, so the registered
decision is `reject-topology-scheduling-and-build-mixture-aware-reference`.
Route-control metadata bytes are not included in the candidate byte number;
the 8.30% saving is therefore not a total-network communication claim.

## Mechanism finding

The failure is not explained by loss of temporal connectivity. Across 40
pulses, V53 deferred at least one formation on 25 pulses (62.5%), deferred
0.975 formations and 1.5 cross edges per pulse on average, and retained both
sensor-level and formation-level temporal strong connectivity on every
window. Mean predicted joint retention risk was only 0.00099 and its maximum
was 0.00599.

Nevertheless, lower communication, lower inter-formation disagreement, and a
small worst-sensor improvement did not translate into better full-network
tracking or cardinality. The remaining action is still too coarse: holding a
formation's entire cross input can protect some labels while simultaneously
withholding useful positive support for other labels. An aggregate one-step
retention score cannot represent these opposing label effects.

This result closes the topology-scheduling branch more strongly than V52:
even exact current-step serve/hold counterfactuals, restricted to cross-
formation inputs and protected by temporal connectivity, do not deliver the
required X36 benefit. The next method must decide influence at receiver--
sender--label granularity and distinguish credible negative evidence from
missing observation opportunity.

## Next experiment

After the feature-gated V54 runtime hooks are installed, run one short smoke
to record active synopsis-label counts, selector runtime, and separate
synopsis/selected-GM ledgers. Then run the frozen X36 convoy oracle arm. It
must improve both full-horizon E-OSPA and cardinality by at least 2%, keep
focus degradation within 0.5%, keep total attempted bytes at or below V46,
and leave no unresolved post-receipt retention violation before any set-GNN
training begins.
