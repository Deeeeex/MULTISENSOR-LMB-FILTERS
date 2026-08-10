# V95 network-budget-reallocated multi-source handover

## Why the baseline comes first

V89 is not an effective routing method. Its matched static physical-tree
baseline obtains E-OSPA 121.277 on M24 versus 121.933 for V89, while X36 gains
only 0.122% and regresses on the weakest formation. V95 therefore treats a
matched static route as a hard comparator rather than a reporting add-on.

Every opened anchor freezes four arms before reading tracking truth:

1. the current physical-tree route held static for the whole horizon;
2. the same residual donors removed for one round, without new targets;
3. a one-round sender-budget reallocation followed by the static route;
4. the same reallocated graph held fixed for the whole horizon.

The third arm must beat both static controls. It must also beat the donor-only
arm, so that improvement cannot be attributed only to deleting inputs.

## The V95 action

The physical-tree reference gives every receiver a 0.70 dominant input and a
0.05 residual input. V95 treats each residual transmission as a network token.
For a sender `s`, its reference residual receiver `d` may donate that token to a
physically reachable target receiver `r`:

```text
remove d <- s (0.05);  donor self weight: 0.25 -> 0.30
add    r <- s (0.05);  target self weight: 0.25 -> 0.20
```

The sender does not change. Consequently the deployable arm preserves the
message count of every sender, the total message count, the 0.70 backbone and
all row-wise weight sums. Attempted and delivered bytes are still measured,
because later posterior evolution can change payload sizes.

At scale `F`, exactly `ceil(F/2)` tokens are selected. Their target formations
and source formations must each be distinct, so direct coverage grows with the
network instead of remaining one gateway wide. Selection maximizes current
target novelty minus current donor-unique support, subject to physicality,
node conflicts, sender uniqueness, strong connectivity and rolling-B3.

The action lasts one round. The remaining horizon equals the current formation
graph diameter plus two recovery rounds. Thus the design tests whether several
new sources injected in parallel can propagate through the unchanged dominant
backbone without repeated broadcasts or moving 0.70 trust.

## Frozen decision gate

The structural gate runs without truth. Only if both M24 and X36 satisfy exact
sender-message parity, scale-proportional source/target coverage, instantaneous
strong connectivity and rolling-B3 may the matched tracking screen run.

The tracking gate then requires, independently on both scales:

- at least 5% mean E-OSPA gain over the canonical static route;
- at least 5% mean E-OSPA gain over the same selected route held fixed;
- positive gain over the donor-only ablation;
- no regression for the worst sensor, weakest formation, consensus tails or
  attempted bytes.

Failure on either scale rejects V95. Opened anchors cannot support validation
or generalization claims.

## Scene-transfer decision after the matched-static gate

The method is frozen before any new scene outcome is opened. If and only if
both braided-handover scales pass the four-arm gate, the unchanged selector,
token count rule and safety projection move to the already geometry-validated
M24/X36 convoy and relay families. Each style must beat its own matched static
route at both scales; a favorable style cannot compensate for a failed one.

Merge-split and curved-corridor follow as development mechanisms: the former
tests a changing physical graph, while the latter isolates changing sensing
ownership on a smoother route. Orthogonal crossing remains stress-only until
its separate observability gate is repaired. If V95 fails the current
cross-scale matched-static gate, no scene expansion is authorized; the next
step is a method redesign rather than a larger result matrix.

## Opened matched-static decision

V95 fails its frozen four-arm gate. The comparison uses identical cached
posteriors, measurements, link uniforms, filter random state, communication
budget and horizon for every arm.

| Scale | Static | Donor only | One-round dynamic | Fixed reallocation | Dynamic / static | Dynamic / fixed | Gate |
|:--|--:|--:|--:|--:|--:|--:|:--:|
| M24 | 125.596200 | 125.596211 | 125.595824 | 125.699438 | +0.000299% | +0.082430% | fail |
| X36 | 131.503832 | 131.473215 | 131.171915 | 130.945110 | +0.252401% | -0.173206% | fail |

The M24 effect is numerically negligible and its consensus metric regresses by
0.199%. X36 shows a small positive effect over the static route, but holding
the same reallocation fixed is better than returning to the static tree; that
fixed arm also regresses the weakest sensor and formation. The experiment
therefore provides no evidence that the dynamic return decision adds value.

No scene transfer or full-episode run is authorized. The current operational
reference remains the static physical tree. A successor must change the method
premise rather than tune V95 on these opened anchors: it needs a horizon-aware
value model that jointly chooses recipients, trust mass and persistence, and
must again be judged first against the matched static and fixed-route controls.
