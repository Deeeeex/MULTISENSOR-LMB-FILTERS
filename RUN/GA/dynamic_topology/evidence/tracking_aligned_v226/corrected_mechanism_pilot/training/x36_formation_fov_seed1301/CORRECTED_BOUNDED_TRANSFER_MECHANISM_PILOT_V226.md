# Corrected V226 bounded-transfer mechanism pilot

- Scene / seed / split: `x36-formation-fov / 1301 / training`
- Source / pilot commits: `19a2484a753505108dd3eb5e3be37db702e9ed0b / 5fe8dd8478fc1f6e4b8ad7a57b8e3ad9d5102709`
- Ordinary reference: `direct-graph-current-physical-reference-v214-v1`
- Fixed static-route baseline included: `0`
- Any joint tail-safe byte-positive candidate: `1`
- Any aggregate-core-positive byte-positive candidate: `1`
- Stage-best rule: Among candidates with positive aggregate E-OSPA, RMSE, window consensus, terminal consensus and attempted-byte saving, maximize the minimum of those five gains; break exact ties by the largest minimum registered tail margin. This reporting record does not relax the registered strict gate.

| Window | Donor | Row | Beneficiary | Source / label | Joint E / R / C / terminal | Worst sensor E / R | Minimum formation E / R | Byte saving | Gate |
|--:|--:|--:|--:|:--|:--|:--|:--|--:|:--:|
| 1 (t=50) | F2 | 3 | F5 | S31 / `[1,3]` | `+0.006 / +1.232 / -0.134 / +0.000%` | `+0.000 / +8.446%` | `+0.000 / +0.000%` | +0.136% | 0 |
| 1 (t=50) | F2 | 1 | F5 | S3 / `[1,3]` | `+0.008 / +2.057 / -0.153 / -0.007%` | `+0.000 / +11.013%` | `+0.000 / +0.000%` | +0.050% | 0 |
| 1 (t=50) | F2 | 2 | F5 | S23 / `[19,15]` | `+0.006 / +1.942 / +0.035 / +0.052%` | `+0.000 / +6.239%` | `-0.000 / -0.003%` | +0.929% | 1 |

## Best strict-gate bounded candidate

Window 1 at t=50, donor F2, beneficiary F5, S23 / `[19,15]`, byte saving `+0.929%`.

## Stage-best aggregate candidate (reporting only)

Window 1 at t=50, donor F2, beneficiary F5, S23 / `[19,15]`, aggregate score `+0.006%`, byte saving `+0.929%`, strict gate `1`, violations `0`.

| Metric | Gain | Registered tolerance | Margin |
|:--|--:|--:|--:|
| `mean_eospa_gain_percent` | `+0.006%` | `+0.000%` | `+0.006%` |
| `mean_rmse_gain_percent` | `+1.942%` | `+0.000%` | `+1.942%` |
| `consensus_gain_percent` | `+0.035%` | `+0.000%` | `+0.035%` |
| `terminal_consensus_gain_percent` | `+0.052%` | `+0.000%` | `+0.052%` |
| `receiver_formation_eospa_gain_percent` | `+0.033%` | `-1.500%` | `+1.533%` |
| `receiver_formation_rmse_gain_percent` | `+9.067%` | `-1.500%` | `+10.567%` |
| `minimum_affected_sensor_eospa_gain_percent` | `+0.012%` | `-5.000%` | `+5.012%` |
| `minimum_affected_sensor_rmse_gain_percent` | `+2.517%` | `-10.000%` | `+12.517%` |
| `worst_sensor_eospa_gain_percent` | `+0.000%` | `-1.500%` | `+1.500%` |
| `worst_sensor_rmse_gain_percent` | `+6.239%` | `-1.500%` | `+7.739%` |
| `minimum_formation_eospa_gain_percent` | `-0.000%` | `-1.500%` | `+1.500%` |
| `minimum_formation_rmse_gain_percent` | `-0.003%` | `-1.500%` | `+1.497%` |

This row is retained for method comparison even when a local-tail tolerance is missed. It does not pass the strict gate unless `strict gate` equals 1.

## Evidence boundary

This corrected-code pilot is a paired H=3 mechanism screen. Donors are teacher-ranked with realized H=3 outcomes; beneficiary, source, label and shortlist features are current-state causal. The ordinary reference is the V214 current-physical route, not the fixed static route required by the end-to-end paper comparison. A positive pilot authorizes policy design only; it is not deployment, generalization or paper-level evidence. The stage-best record is retained for method decisions even when a registered local-tail tolerance is missed; it is reporting-only and never changes strict-gate eligibility.
