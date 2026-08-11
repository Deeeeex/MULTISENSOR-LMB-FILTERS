# V103 finding: mean headroom exists, whole-posterior handoff is unsafe

## Matched result

| Arm | Mean E-OSPA | Gain over static | Attempted-byte saving |
|:--|--:|--:|--:|
| Static full payload, H=8 | 84.037151 | -- | -- |
| V103 maturity-aware handoff | 79.554740 | +5.334% | +5.981% |

Per-step gains are
`[1.216, 2.115, 5.061, 6.538, 5.553, 7.832, 7.642, 6.997]%`.
The minimum after the first handoff is 5.553%, window and terminal consensus
improve by 9.651% and 18.432%, and every rolling sensor- and formation-level
B3 window passes.  Candidate runtime is 244.01 seconds versus 251.51 seconds
for static.  The phase-corrected pipeline therefore creates real network
headroom without extra communication or computation.

## Why the strict gate still fails

Formation gains are `[-0.945, 4.676, 7.731, 9.079, 11.740, -0.022]%`.
The network mean hides two incompatible receiver groups:

- F1 improves through t=76 but falls to -5.868%, -8.603% and -15.752% over
  t=77--79 after its t=75 handoff.  All six F1 sensors regress at t=79.
- F6 gateway sensor 32 improves 6.458% at t=79, but peers 31 and 33 regress
  by 7.735% and 7.757%.  The five non-gateway peers regress by 2.948% on
  average; sensors 34--36 are approximately neutral.

By contrast, most F3 and F5 receivers benefit strongly, and F2/F4 contain a
mixture of positive and negative peers.  The sign changes within one
formation prove that formation membership, gateway age and graph reachability
cannot determine transport value by themselves.

## Mechanism decision

V101 established that protection can create a useful gateway posterior.
V102 established that broadcasting before maturation cannot transport it.
V103 establishes that transporting a matured *complete* posterior can improve
the network mean while harming specific peers.  Together these results close
the remaining topology-only explanation:

```text
protect -> mature -> handoff
                     |
                     +-- useful only for selected receiver-label inputs
```

The next controlled object is the label-wise effective KLA graph on top of the
same physical carrier.  The successful formation controller decides when a
gateway is eligible; a receiver--sender--label value layer decides which parts
of that gateway posterior may participate at each peer.

This successor must not reuse two previously rejected objectives:

1. V54/V55 minimized divergence from full-message fusion, which preserves an
   estimator but does not seek tracking improvement;
2. V61/V62 used one-step sender-support heuristics, which did not reproduce
   the temporal formation benefit and created recursive payload cost.

The new teacher must instead predict downstream tracking regret of feasible
label subsets after a matured handoff.  Online inputs remain causal and
truth-free; target truth and future outcomes are allowed only to label offline
teacher rollouts.  A deterministic projector must preserve receiver-supported
labels, communication budget and rolling information flow, with the static
label input as the fallback.

No GNN training is authorized until a bounded exact receiver-label oracle
shows at least 5% strict headroom on both an M24 and X36 development window.
This prevents learning another coherent but task-misaligned compression rule.
