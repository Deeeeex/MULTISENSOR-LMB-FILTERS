# M24 paired causal H=3 option sentinel

- States / registered options per family: `4 / 8`
- Reference / one-step oracle / persistent oracle mean E-OSPA: `15.271324 / 15.014760 / 14.563902`
- One-step / persistent gain: `+1.680% / +4.632%`
- Incremental persistent gain: `+2.952 pp` (gate `>=2.0 pp`)
- States improving one-step oracle: `1 / 4` (gate `>=2`)
- Strong non-outlier states: `1 / 3` (gate `>=1`)
- Hard audit / headroom / full-grid gates: `0 / 0 / 0`

| Seed-time | Source | Ref. | One-step | Persistent | Persistent option | Gain vs ref. | Incremental | Worst | Consensus | Repair p/o/r | Rows pair/p/o | Delivered B3 s/f | Safe |
|:--|:--|--:|--:|--:|:--|--:|--:|--:|--:|:--:|--:|:--:|--:|
| 11-78 | learned | 14.983228 | 14.983228 | 14.983228 | reference-schedule-live-h3 | +0.000% | +0.000 pp | +0.000% | +0.000% | 0/0/0 | 6/0/2 | 0.33/0.33 | 0 |
| 19-78 | ccw | 21.026617 | 21.026617 | 21.026617 | reference-schedule-live-h3 | +0.000% | +0.000 pp | +0.000% | +0.000% | 0/0/0 | 3/2/3 | 0.00/0.00 | 0 |
| 23-78 | ccw | 8.427007 | 8.427007 | 5.961354 | deterministic-burst-live-h3 | +29.259% | +29.259 pp | +40.949% | +34.236% | 0/0/0 | 5/0/3 | 0.67/0.67 | 0 |
| 27-78 | learned | 16.648442 | 15.622186 | 16.284410 | diverse-analytic-live-h3 | +2.187% | -3.978 pp | +10.413% | +3.303% | 0/0/0 | 5/0/2 | 1.00/1.00 | 0 |

## Decision

Persistent causal options are rejected before full-grid evaluation and selector fitting.

## Boundary

This paired opened-training sentinel compares eight persistent causal H=3 options against the same eight one-step-then-reference controls on four registered t=78 behavior states. Oracle comparison is restricted to the same paired-completed action rows after exact first-graph and first-weight matching. Every completed arm is truth-free and selects 24 messages per step; later policy-reported repair may not exceed its paired reference. This flag is not a universal proposal-to-selected rewrite diagnostic. Source behavior restart hashes are replayed exactly. The source states come from 40-message CCW or learned behavior trajectories, so this does not establish steady-state 24-message performance, recursive feasibility, or formation-FoV M24/X36/X48 gains. Centralized coordinator metadata bytes are not included in communication cost.
