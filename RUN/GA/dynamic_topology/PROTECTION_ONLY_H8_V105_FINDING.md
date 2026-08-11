# V105 finding: the H=8 protection schedule explains both gain and harm

## Matched result

| Arm | Mean E-OSPA | Gain | Min. formation | F6 peer terminal | Bytes saving |
|:--|--:|--:|--:|--:|--:|
| Static H=8 | 84.037151 | -- | -- | -- | -- |
| V103 broad handoff | 79.554740 | +5.334% | -0.945% | -2.948% | +5.981% |
| V104 receiver oracle | 79.555155 | +5.333% | -0.931% | -2.945% | +6.091% |
| V105 protection only | 79.617863 | +5.259% | -0.931% | -2.940% | +6.117% |

V105 reuses the exact frozen static outcome and executes only the candidate.
It applies the V103 eight-step control-only protection schedule while keeping
every adjacency and fusion-weight row on the static fixed-counter-clockwise
route.  It performs no gateway handoff.  The candidate improves every return
time, reaches at least 5.188% gain from t=76 onward, improves window and
terminal consensus by 9.650% and 17.214%, and saves 6.117% attempted bytes.

## Causal conclusion

Removing every handoff changes the V103 network gain by only 0.075 percentage
points.  It also leaves the two local failures essentially unchanged:

- F1 remains the weakest formation at -0.931%;
- F6 remains slightly negative at -0.021%;
- the five F6 non-gateway peers still regress by 2.940% at the terminal time.

The H=8 protection schedule is therefore sufficient to reproduce both the
network mean headroom and the local harm.  Gateway handoff and receiver-row
selection are not the primary explanation for either effect in this window.
V103 is numerically the best network-mean arm, but its 0.075-point advantage
over the simpler V105 arm does not resolve any safety failure.  No current
X36 arm can be called a stable winner over the matched static baseline.

## Method decision

The useful object is no longer dynamic physical routing.  It is the temporal
controller that decides which formations use control-only payloads and for how
long.  The present controller expands from `[1 2 4 5]` to all six formations
and never deactivates within the H=8 window.  That long-lived blanket action
creates network-level benefit but sacrifices F1 and the F6 peers.

The next design must therefore make protection activation and deactivation
risk-aware before adding handoff, label selection or a GNN.  A bounded next
screen should vary only the protection lifetime and release rule while keeping
the route, cache, random stream and matched static baseline frozen.  It should
optimize the strict objective: at least 5% network gain, nonnegative gain for
every formation and sensor group, positive F6-peer terminal gain, and no extra
communication.  Receiver-label learning remains closed until such a temporal
controller demonstrates safe headroom.

