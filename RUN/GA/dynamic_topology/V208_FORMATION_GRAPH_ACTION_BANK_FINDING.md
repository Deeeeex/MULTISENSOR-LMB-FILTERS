# V208 formation-graph action-bank finding

## Scope

This is development evidence from the already opened X36 formation-FoV
trajectory at seed 211.  The action bank and every model input are built from
current posteriors, geometry, physical links, causal route history, and byte
accounting.  Truth is attached only after an action is frozen to score its
current-state effect.  The result is not recursive, held out, cross-seed, or
cross-scene evidence.

## Search-space preflight

The diverse truth-free shortlist reduces the supported-label KLA bank from a
mean of `3751.9` to `103.9` candidates per page (`2.769%` retained).  Despite
that reduction, all five routing keys later used by the V206 mechanism teacher
remain in the shortlist.  This establishes search-space coverage only; the
teacher keys are read after shortlisting and never enter the ranking features.

## Dense t=72 action values

The first complete block contains 108 shortlisted and executable actions.
All 108 fit the same-page communication credit after charging both the light
synopsis and the complete-label payload.

| Criterion | Count |
|:--|--:|
| Mean E-OSPA and RMSE both improve | 21 / 108 |
| Mean E-OSPA, RMSE, and consensus improve; byte credit remains positive | 15 / 108 |
| Above conditions plus positive receiver-formation E-OSPA/RMSE and no affected-sensor or network-worst regression | 5 / 108 |

The known V206 action `F2 <- S19, label [13,12]` is recovered without using
truth in construction.  Its one-step values are:

| Mean E-OSPA | Mean RMSE | Consensus | Net byte saving | Min affected E-OSPA | Min affected RMSE |
|--:|--:|--:|--:|--:|--:|
| +0.932% | +7.163% | +2.440% | +5.180% | +3.913% | +63.671% |

It is the strongest zero-tolerance tail-safe action in this block.  A second
source, `F2 <- S24` for the same label, has nearly identical tracking and
consensus value and slightly more byte credit, showing that the useful action
is not identified only by one memorized sensor key.

## Why a scalar mean reward is unsafe

The two largest mean tracking actions target F5 with the same `[13,12]` label.
They improve mean E-OSPA by about `0.96%` and mean RMSE by about `7.93%`, but
worsen consensus by about `0.44%` and worsen the minimum affected-sensor RMSE
by about `143%`.  A controller trained or ranked only on mean E-OSPA/RMSE would
prefer these actions over the V206 teacher and recreate the tail failure that
the method is meant to remove.

The dense block therefore changes the V208 target contract:

1. predict E-OSPA, RMSE, consensus, and byte value separately;
2. predict receiver-formation and affected-sensor tail values, rather than
   deriving safety from the aggregate score;
3. use conservative lower bounds as an admission screen;
4. rank only the admitted actions, with no-op as the default;
5. keep the strict all-tail-positive condition as a strong evidence label,
   not as the only paper-level usefulness criterion.

## Observable structure and next test

On this single block, joint-positive actions tend to have shorter normalized
source-to-receiver distance, higher receiver-source compatibility, and higher
source observation opportunity than rejected actions.  These are descriptive
development effects, not validated thresholds.  The next evidence gate is the
same dense construction over all eight X36 pages, followed by a leave-one-page
out learnability check.  Only after the representation can recover positive
actions without truth will recursive release actions and held-out trajectories
be opened.
