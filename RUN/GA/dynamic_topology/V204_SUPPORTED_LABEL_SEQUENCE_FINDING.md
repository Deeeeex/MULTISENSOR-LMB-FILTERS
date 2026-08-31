# V204 supported-label mode sequence finding

## Result

The X36 `seed=211, t=72, H=8` supported-label sequence combines four causal
mechanisms while retaining the one-action-per-page cap:

1. F2 observation handover at `t=72`;
2. deferred F6 precision refresh at `t=73`;
3. F3 observation handover at `t=76/78`, with the redundant `t=79` repeat
   removed; and
4. F1 MAP-sensitive label KLA at `t=77`.

Every applied action sends one complete Bernoulli GM label and uses residual
KLA with source weight `0.5`.  The scheduled F5 action is intentionally not
counted: `[19,16]` has zero receiver support, so the KLA candidate gate rejects
it and only the charged synopsis remains on that page.

| Metric | Static full payload | V99 no repair | Without F1 | With F1 |
|:--|--:|--:|--:|--:|
| Mean E-OSPA | 84.037151 | 79.451115 | 77.330084 | 77.084367 |
| Static-relative E-OSPA gain | 0 | +5.457% | +7.981% | +8.273% |
| Mean RMSE | 59.967347 | 62.172152 | 51.214595 | 50.927938 |
| Static-relative RMSE gain | 0 | -3.677% | +14.596% | +15.074% |
| Consensus gain | 0 | +8.803% | +14.246% | +15.112% |
| Attempted-byte saving | 0 | +5.423% | +3.505% | +3.134% |

Relative to the no-F1 arm, the MAP-sensitive F1 action lowers E-OSPA by
`0.245717`, RMSE by `0.286657`, and consensus error by `0.531325`, while
adding `106,192 B`.  The action is therefore useful despite its low ordinary
Bernoulli-risk score; F1 must remain a separate mode rather than being removed
by a global risk threshold.

## Formation closure

With F1 enabled, formation-relative gains are:

| Formation | F1 | F2 | F3 | F4 | F5 | F6 |
|:--|--:|--:|--:|--:|--:|--:|
| E-OSPA gain | +0.883% | +12.634% | +9.546% | +8.970% | +12.506% | +4.868% |
| RMSE gain | +2.352% | +55.486% | +5.764% | +1.789% | -2.009% | +41.225% |

All formation E-OSPA tails and five of six formation RMSE tails are now
positive.  The only remaining formation failure is F5 RMSE, exactly the mode
whose absent label cannot be repaired by KLA.  This isolates the final
mechanism experiment to one hard support-restoration action at `t=79`.

The sequence improves RMSE, consensus, and communication substantially over
the V187 balanced development best (`53.540189`, `+9.834%`, and `+0.160%`
byte saving), but V187 still has lower mean E-OSPA (`74.678760`) and a less
negative weakest formation-RMSE tail.  V187 therefore remains the main-table
current best until the F5 restore closes the complete gate.

## Method consequence

The supported-label modes are recursively compatible and do not require a
higher per-page action cap.  The causal controller needs:

- semantic-action cooldown to suppress the redundant consecutive F3 repeat;
- per-mode value ranking so F6 can be deferred behind F2;
- an explicit MAP-sensitive mode because ordinary risk ranking misses F1;
- and an operator gate based on receiver support: residual KLA for supported
  labels, protected hard insertion/replacement for absent labels.

## Evidence boundary

This is an opened, ideal-delivery, fully charged teacher result on one X36
trajectory.  Source, label, formation, and page identifiers are teacher
routing keys.  It proves supported-label mechanism headroom, not an online
selector or cross-seed/generalization result.  The F5 action was rejected and
must not be described as applied or validated.
