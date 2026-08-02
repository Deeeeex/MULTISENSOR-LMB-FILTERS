# V37 X36 source-only scale gate

## Why this X36 scene is retained

The same-hardware `x36-formation-fov` scene keeps the M24 sensor contract:
120-degree total FoV, 300 m hard range, shared formation boresight, detection
probability, clutter, noise, and per-sensor quality scaling. On seed 211, the
focus-window visible-target load is 13.3750 for M24 and 13.3291 for X36, a
relative difference of about -0.34%. Both have zero focus-window blackout.

X36 nevertheless increases formations from four to six, sensors from 24 to
36, targets from 16 to 24, focus handovers from 31 to 68, and mean physical
inter-formation edges from 216 to about 493. It therefore tests scale and route
combinatorics without silently replacing the sensor hardware or making nearly
all targets visible.

## Frozen source states

V37 uses X36 seed 211 only and opens one anchor inside each registered paired
blockage window:

| Anchor | Registered blockage pair | Return window |
|--:|:--|:--|
| 72 | formations 1 and 2 | 72--74 |
| 100 | formations 3 and 4 | 100--102 |
| 128 | formations 5 and 6 | 128--130 |

The anchors are fixed before generating any X36 posterior. A single fixed-CCW
reference trajectory produces all three predecision caches. Cache generation,
scene auditing, initial v35 control construction, and later full H=3 source
replay use no tracking truth or future outcome.

## Authorization sequence

1. Reproduce the same-hardware scene and load gates from clean source.
2. Generate and hash the three reference posterior caches in one trajectory.
3. Construct the unchanged v35 controller at each anchor and freeze its exact
   initial source metrics and action.
4. Execute the complete H=3 controller without scoring tracking; freeze every
   suspension, release, fallback, posterior/link-use, and rolling-B3 field.
5. Reproduce those traces from a later clean commit.

Only after all five steps may a separate protocol authorize one fixed
reference-versus-v35 pair per passing state. The later aggregate outcome gate
is unchanged from M24: at least two of three strict-strong states, median mean
tracking gain at least 2%, no state below -1%, and at least two positive
terminal-consensus states. This design does not yet authorize any X36 outcome,
GNN training, X48, reserved seed, or validation claim.
