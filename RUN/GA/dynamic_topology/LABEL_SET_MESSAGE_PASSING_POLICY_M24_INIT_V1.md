# M24 label-set message-passing initialization

- Generated: 2026-07-30 06:21:08
- Commit: `2db787c536903fd8727d5e289dd50565aa3f349d`
- Protocol: `m24-label-set-simulator-policy-redesign-v1`
- Contract: `label-set-two-round-message-passing-policy-m24-v1`
- Training pairs: `144`
- Training pair preference accuracy: `0.8194`
- Training difference Spearman: `0.7343`
- Training mean absolute difference error: `0.013950`
- Safe projector required: `1`
- Closed-loop return evaluated: `0`
- Policy selected / deployable: `0 / 0`
- Development / held-out M24 / X36 evaluated: `0 / 0 / 0`

## Boundary

This artifact is a current-task supervised initialization on already-opened M24 training states. Its inputs are truth-free, but its initialization targets use current tracking truth. The two-round scorer must remain behind the deterministic safe residual-cycle projector. No paired closed-loop return has been optimized or evaluated, so the artifact is not a selected policy and supports no M24 or X36 performance claim.
