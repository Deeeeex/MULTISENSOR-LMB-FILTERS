# V65 M24 network-additive attribution result

The frozen X36-derived V65 selector was evaluated on ten already-opened M24
posterior states.  Observable features and decisions were constructed before
joining prior best-known three-step gains, and those older gains are not
reported as V65 outcomes.

| Group | Count | V65 behavior |
|:--|--:|:--|
| Prior strong gain, at least 5% | 2 | `2/2` trigger |
| Prior weak gain, below 2% | 6 | `6/6` reference fallback |
| Intermediate | 2 | conservative fallback |

The network risk strictly separates the registered strong and weak groups.
The strong radial states are t=104 (`2.515%`, set `[1,3]`) and t=124
(`2.099%`, set `[2,3]`).  Every opened convoy state remains below `0.455%`,
including the earlier false-positive t=40 state, which now falls back.

This establishes cross-scale event-selection headroom without tuning the 1%
budget on M24.  The required next evidence is direct paired V65 tracking at
t=104 and t=124; low-risk fallback states do not require redundant tracking
runs.
