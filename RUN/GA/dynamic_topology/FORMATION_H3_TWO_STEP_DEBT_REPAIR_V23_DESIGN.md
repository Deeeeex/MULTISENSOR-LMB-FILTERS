# Formation H=3 two-step debt-repair beam v23

## Question

v22 shows that the first action creates several kinds of terminal debt.  A
high tracking return can coexist with a formation-tail loss, a sensor-tail
loss, consensus divergence, or extra attempted and delivered bytes.  v23 asks
whether a second action can repay those debts while retaining at least `3%`
mean tracking gain.

## Frozen beam

The first step uses six v22 prefixes with distinct, predeclared roles:

| Prefix | Role |
|:--|:--|
| `[1,1,1,1]` | all-reference control |
| `[1,4,1,1]` | sole weak-safe v22 anchor |
| `[1,1,2,2]` | balanced prefix with low byte debt |
| `[1,1,2,4]` | balanced prefix closest to the strong-gain boundary |
| `[1,1,3,4]` | maximum-mean prefix with consensus and byte debt |
| `[1,4,3,1]` | high-mean prefix with byte credit and consensus debt |

The second step is a structured 18-vector factorial grid.  Formation 1 stays
at reference because every v22 interaction involving it was dominated or
unsafe.  Formation 2 chooses `{reference, mode 4}` to create communication
and worst-sensor slack.  Formations 3 and 4 independently choose
`{reference, mode 2, mode 4}` to retain tracking value or repay consensus.
The third step is always reference.  The full Cartesian beam therefore
contains `6 x 18 = 108` sequences and is frozen before execution.

## Gates

- all six v22 prefix-plus-reference outcomes must reproduce within `5e-6`
  percentage points;
- every action must pass physical, payload, exact-execution, rolling-B3,
  truth-use, repair, emergency, and infeasibility gates;
- strict feasibility requires all six signed targets to be nonnegative;
- strong headroom requires strict feasibility and at least `3%` mean
  tracking gain.

If strong headroom exists, the resulting sequence can seed a multi-head graph
value model whose outputs separately predict mean gain, two tail risks,
consensus debt, and two byte debts.  A hard projection must still reject any
candidate with a negative lower-confidence bound and retain the reference
fallback.  If no strong sequence exists, GNN training remains unauthorized:
the present action family has not shown a safe teacher target.

This is an opened single-state mechanism probe.  Seeds 223/227, X36, and
final seeds remain unopened.
