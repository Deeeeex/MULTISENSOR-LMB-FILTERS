# Stronger evidence before creating a persistent identity conflict

The restored S candidate passed the original development gate, but its
sequence-macro OSPA gains were only 0.75% and 0.19%. Forty-three logged
branch events include a low-confidence branch near no scored object and
several cases where the two conditional detection centers lie on opposite
sides of the same annotated vehicle. These observations come entirely from
the already exposed nine-segment development set. The complete S assessment
continues unchanged; its remaining V2V results have not been consulted to
choose the settings below. The additional 14-segment test cohort remains
unread.

Freeze a two-factor comparison with exactly three new candidates:

| Candidate | Required current majority confidence at each endpoint | Nominal discrepancy quantile |
|---|---:|---:|
| Q: `assoc_quality` | 0.9 | 0.99 |
| N: `assoc_nis` | 0.5 | 0.999 |
| QN: `assoc_quality_nis` | 0.9 | 0.999 |

S is the fourth cell of the comparison: confidence 0.5 and quantile 0.99.
The 0.999 quantiles for two, four and six degrees of freedom are
13.815510557964274, 18.46682695290317, and 22.457744484825326.
These remain nominal decision thresholds, not calibrated false-alarm
probabilities. The required sample count stays at least two qualified
received samples in the current and preceding two frames.

All three use S's unchanged persistent-state transitions, original free-pair
Gaussian KL cost, deterministic branch identifiers, source-specific
abstention, recursive filter and 416-byte component packet. The quality
criterion controls which current samples enter the evidence calculation.
The chosen quantiles control both the discrepancy-based entry and clearance
tests. No detector, score calibration, local update, kinematic model,
likelihood-ratio admission equation, input, seed, or packet-loss schedule
changes. The corrected column-shaped division is used for one-to-many
inputs and has exact old-case parity; it is an implementation repair.

First replay 0001, 0006 and 0007 with original GCE, restored S, Q, N and QN.
These exercise the successful split, the low-confidence branch, and the
detection-center offset/communication-gap case. Require exact parity with
the saved GCE and S trajectories and independently verify all decisions.
Then complete the other six development segments with Q, N and QN.

Choose among the three new candidates only after all nine segments and
both link conditions have exited and passed their audits. A candidate must
reduce sequence-macro OSPA by at least 1% in each condition versus original
GCE, while not increasing the pooled scored 2 m wrong-pair rate. Select the
lowest two-condition mean among eligible candidates, with exact ties
preferring Q, then N, then QN. Retain all earlier restored results in the
comparison. Report both directions of output identity changes, missed
common associations, GOSPA components and actual bytes even when they do
not favor the selected candidate.

An advancing candidate requires matched GCE/guarded-scalar fusion controls,
the complete exposed release, and the additional frozen test cohort before
paper promotion. The additional test data will not be used to tune these
settings. If no candidate clears the stronger development gate, record
that result and keep the additional test cohort unexposed.
