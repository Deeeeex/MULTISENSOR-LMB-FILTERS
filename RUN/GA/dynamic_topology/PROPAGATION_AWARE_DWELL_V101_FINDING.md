# V101 finding: holding fixes interruption, not transport

## Matched result

| Arm | Mean E-OSPA | Gain over static | Attempted-byte saving |
|:--|--:|--:|--:|
| Static full payload | 84.581111 | -- | -- |
| V100 online | 80.807326 | +4.462% | +5.250% |
| V101 three-step hold | 80.700047 | +4.589% | +5.811% |

V101 changes only t=76, retaining F5 and F6 after their positive selections at
t=75.  It improves V100 by another 0.133%.  Per-step gains over static become
`[1.216, 2.115, 5.061, 6.145, 5.188, 7.997]%`; therefore the post-propagation
minimum now passes 5%.  The six-step mean and weakest-formation gates do not.

## Spatial mechanism

Formation gains are `[2.265, 2.866, 5.512, 8.367, 8.758, 0.165]%`.  The extra
hold materially improves F5 but leaves F6 unchanged.  F6's by-time gains are
`[0, 0, 0, 0.002, 0.002, 0.980]%`.  At t=77, gateway sensor 32 improves
5.632%; sensors 31 and 33--36 range from -0.014% to -0.004%.

This separates two mechanisms.  A positive-net control-only action can protect
the gateway posterior, and renewing it for the registered within-formation
depth prevents premature interruption.  The installed dominant tree does not,
however, transport that task benefit to the remaining members through repeated
KLA.  Graph reachability is therefore not equivalent to useful posterior
propagation.

## Next method decision

Close longer holds and do not open M24 replication, four-anchor confirmation,
new scenes or a learned selector.  V102 should layer two causal operations on
the same carrier budget:

1. suppress the harmful cross-formation residual input at a positive-net
   gateway, as in V100/V101;
2. on the following page, promote that protected gateway into the existing
   dominant within-formation input of its peers while demoting, rather than
   deleting, the displaced sender so row message counts and weights remain
   unchanged.

V86/V89 tested broadcasting after a sparse low-weight information-acquisition
substitution and lacked network coverage.  V102 is a distinct composition:
V100 already protects multiple formations with 4.589% network headroom, and
the broadcast addresses the directly observed F6 transport failure.  Only if
this combined action lacks headroom should the method move to signed
receiver--sender--label exceptions or a GNN approximation.
