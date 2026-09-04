# V268 source-preserving label packet

## Method decision

V266 showed that a carefully bounded label input can materially improve
localization, but V267 showed why the same action is unsafe when it is fused at
the intermediate relay.  The relay's immediate existence probability can pass
the eta guard and still alter the posterior that is propagated through later
KLA rounds.  Cardinality loss then appears one or more pages after the guarded
action, outside the scope of any one-step receiver check.

V268 removes that unintended state transition.  A selected complete
Bernoulli-GM label is sent from the donor sensor to the selected relay, but the
relay stores it as a source-identified packet and does not fuse it into its own
LMB.  On the next page, the stored source density is predicted once with the
registered survival and motion model, forwarded over a second real physical
link, and inserted only into the intended target formation's ordinary
label-wise KLA.  A current V242 backbone edge from the relay to the target is
preferred; a currently physical target link is the deterministic fallback.

The final receiver still uses the frozen V266 share grid
`[0.05, 0.025, 0.0125, 0.00625]` and the V267 asymmetric existence guard.  The
source object is always appended as a distinct provenance-preserving input,
even if the same physical source also appears among ordinary inputs.  Thus the
KLA consumes the delayed original source density, not a relay-modified or
silently substituted density.

## Communication and causality contract

- Both first and second physical hops are charged as complete-label payloads.
- Two extra scalar fields carry original-source identity and source time; the
  standard message header and label fields remain charged by the existing
  payload estimator.
- The packet has exactly one page of registered age.  It is predicted once,
  and a blocked or dropped second hop is not retried.
- First-hop delivery only writes the packet cache.  It cannot change the
  relay's LMB, state estimate, or outgoing ordinary posterior.
- The second-hop receiver performs at most one bounded label-wise KLA action.
- The source/formation/label decision remains based on current posterior and
  current physical context.  No target truth, future measurement, or future
  tracking outcome is available to the controller.
- The compact centralized risk synopsis is still excluded, so this remains a
  mechanism screen rather than deployable end-to-end cost evidence.

## Frozen M24 screen

- Scene / seed: `m24-formation-fov-temporal-coupled-formation-braid / 1301`.
- Continuation: paired V242 state at `t=57`, evaluated through `t=73`.
- Full-posterior backbone: unchanged V242, 30 ordinary messages per page.
- Packet age: one page; no retry.
- Final source-share grid: `[0.05, 0.025, 0.0125, 0.00625]`.
- MAP-positive guard: never cross `0.5`, at most `0.25` log-odds loss.
- MAP-negative guard: no additional log-odds loss.

The screen passes only if the F4 event E-OSPA and RMSE both improve, network
E-OSPA/RMSE/consistency stay within the registered regression envelope, no
formation violates its safety envelope, and the spliced episode still saves
bytes against corrected static routing.  A pass authorizes one full M24 run.
A failure closes this source-preserving packet source/beneficiary pairing;
threshold sweeps or a learned ranker are not justified by a failed causal
action.
