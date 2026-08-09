# V56 tracking-aligned atomic-action M24 result

## Decision

The V56 formation-local, one-step action bank fails the frozen M24 headroom
gate. Its constrained three-state oracle mean is `+0.493940%`, compared with
the required `+5%`. This closes the atomic bank as the final action space and
does not authorize graph-model training. The run stopped before X36 because
the joint M24/X36 gate was already impossible to pass.

The result does not reject tracking-aligned topology control. It shows that a
single formation changed for one step is too weak as the deployed decision.
The observable source/trust and protection modes remain useful primitives for
a coordinated, temporally adaptive successor.

## Frozen evidence

- Generation commit: `f02c21f2b0e67594bd8473d2a96346d7dab47b42`
- Preset / seed: `m24-formation-fov-convoy / 1201`
- Anchors: `[60, 100, 132]`
- Horizon: `3`
- Intervention duration: `1`
- Mean-gain gate: `5%`
- Positive-state gate: `2/3`
- X36 outcomes opened: `0`

| Time | Available actions | Best tracking oracle | Best constrained oracle | Constrained action |
|--:|--:|--:|--:|:--|
| 60 | 8 | +1.312854% | +1.312854% | formation 1, source/trust 0.70 |
| 100 | 5 | +0.168953% | +0.168953% | formation 2, protect cross input |
| 132 | 5 | +0.104101% | +0.000012% | formation 4, protect cross input |
| **Mean** | — | **+0.528636%** | **+0.493940%** | — |

The constrained oracle requires nonnegative cardinality, minimum-formation,
worst-sensor, consensus, and attempted-byte targets. All three states have a
strictly positive constrained action, so the coverage gate passes `3/3`; the
mean-gain gate fails by `4.506060` percentage points.

## Mechanism finding

At `t=60`, only formation 1 exposes safe source/trust changes. Trust 0.70
improves mean tracking by `1.313%`, consensus by `0.537%`, and attempted bytes
by `0.683%`. This is a coherent but scale-diluted signal: changing one of four
formations improves the network, yet cannot deliver a five-percent network
gain in one step.

At `t=100` and `t=132`, no source/trust change survives the current local
safety projection. Protection actions mainly save communication. Their best
strict tracking gains are `0.169%` and approximately zero. Expanding only the
number of single-formation protection actions therefore cannot close the gap.

## V57 direction

The next method must change the decision object rather than relax the gate:

1. test whether the useful `t=60` source/trust action compounds when held for
   two or three steps;
2. represent one mode per formation and project the complete heterogeneous
   mode vector, rather than rejecting a joint action whenever one isolated
   local mode fails;
3. use a bounded beam or structured scorer to compose source/trust and
   protection primitives across formations;
4. make hold and recovery timing state dependent, with physical, payload,
   label-retention, and rolling information-flow constraints outside the
   learned model; and
5. require the redesigned M24 headroom bank to clear the unchanged `5%` gate
   before opening X36 outcomes.

## Evidence boundary

These are opened-development convoy results from one seed and three anchors.
They reject the frozen V56 atomic action space, but they are not M24
validation and make no claim about X36 performance or multi-scene
generalization.
