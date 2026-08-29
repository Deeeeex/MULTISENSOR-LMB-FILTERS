# V146 M24 finding: whole-posterior edge-role scheduling is closed

V146 evaluates the preregistered seven-action, scale-equivariant bank of
single-payload R/W edge-role vectors on the frozen M24 development state.  No
action passes the independent M24 gate, so the sequential protocol keeps all
matching X36 screens sealed.  This is a repository-only negative result and
must not be presented as a paper result.

## Frozen results

- Source commit: `c3fae0b`.
- Preset / seed / anchor / protection action:
  `m24-formation-fov` / `1601` / `95` / `25`.
- Temporal envelope: `R-W-W-W-R-W-R-...` on selected protected
  intra-formation receivers; every cross-formation edge carries R.
- Every attempted edge carries exactly one current mixture-aware LMB
  posterior; no auxiliary payload is sent.

| Role bank action | Formation ranks | Intervention | Full window | Mature window | Minimum sensor | Minimum formation | Attempted bytes | Pass |
|:--|:--|--:|--:|--:|--:|--:|--:|:--:|
| first | `1` | +3.975% | +2.936% | +2.936% | -5.224% | -4.224% | +2.648% | no |
| last | `4` | +3.181% | +1.233% | +1.135% | -11.395% | -9.469% | -0.797% | no |
| odd | `1,3` | +4.019% | +6.079% | +6.583% | -1.281% | +0.294% | +2.519% | no |
| even | `2,4` | +3.523% | +1.796% | +1.728% | -10.304% | -8.228% | -1.764% | no |
| upper half | `1,2` | +4.011% | +1.068% | +0.865% | -6.550% | -5.661% | -1.461% | no |
| lower half | `3,4` | +3.309% | +4.488% | +4.888% | -0.029% | +1.371% | +1.231% | no |
| all | `1,2,3,4` | +3.670% | +1.832% | +1.963% | -8.228% | -3.727% | -1.700% | no |

All seven arms rejoin the reference relay state exactly (`100%`) and finish
without execution failures.  The failure is therefore not caused by an
incomplete run or a broken recovery path.

## Decision

The best intervention gain is only `+4.019%`, below the registered `+5%`
threshold.  That same odd-rank action has useful full/mature gains
(`+6.079% / +6.583%`) but harms the weakest sensor by `-1.281%` and increases
charged bytes by `+2.519%`.  The locally safest lower-half action still misses
the intervention gate (`+3.309%`), the minimum-sensor tolerance
(`-0.029% < -0.010%`) and the no-byte-increase gate (`+1.231%`).  No action
offers a hidden safety--utility trade that justifies opening X36.

V143--V146 collectively show that multiplexing two whole-posterior lineages is
too coarse: it moves many labels and mixture components together when the
causal evidence points to a small number of time-sensitive, spatially
supported labels.  Per the frozen V146 protocol, learned R/W role scheduling
is closed before teacher generation or GNN training.

This does not prove that every possible posterior-routing policy must fail.
It closes this registered whole-posterior action space on the opened
development state.  The next method should retain one standard mixture-aware
LMB posterior per node and use observable label-support age to choose which
feasible directed edges carry it under the static attempted-message and exact
serialized-byte budgets.
