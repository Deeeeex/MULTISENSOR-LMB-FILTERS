# V193-guided releases repair the predicted X36 formations

## Paired single-release results

V193 identified X36 F2 and F5 from current observable set-entry risk before
these release outcomes were generated.  V191 then restored the ordinary full
posterior to one identified formation at t=72 while keeping all other V99
decisions, links, measurements, fusion weights and filter RNG paired.

| Arm | E-OSPA gain | RMSE gain | Consensus gain | Byte saving | Target formation RMSE gain |
|:--|--:|--:|--:|--:|--:|
| V99 | +2.802% | -0.666% | +5.149% | +6.550% | F2 -14.198%; F5 -1.000% |
| Release F2 at t=72 | +2.181% | +0.589% | +2.648% | +5.397% | F2 0% |
| Release F5 at t=72 | +2.276% | -0.583% | +5.333% | +6.365% | F5 +0.547% |

Each release repairs the formation predicted by V193.  Releasing F2 turns the
network-level RMSE gain positive.  Releasing F5 flips the F5 local RMSE gain
positive but leaves the much larger untreated F2 gap, so the network mean
remains slightly negative.  Both actions preserve positive E-OSPA,
consensus and attempted-byte savings.

## Decision

The support-aware set-entry risk is now supported by both observable
separation and causal action headroom on M24 and X36.  The next candidate is
an online safety projection that:

1. computes the V99 positive-net omission proposal;
2. evaluates the V193 unsupported-entry condition for each proposed
   formation on the current state;
3. removes every risky formation from that page's omission set;
4. executes the remaining omission set using the unchanged message builder
   and byte ledger.

The automatic policy must be tested jointly because F2 and F5 can be released
on the same page and later V99 decisions are state dependent.  The individual
teacher gains cannot be added arithmetically.  Even a successful joint result
will still leave X36 F3's -1.489% formation RMSE gap, because F3 is not an
omission action at the first page; that residual belongs to the later sparse
repair layer rather than the release layer.

## Evidence boundary

The release formation identifiers are teacher inputs and the opened seed-211
outcomes are development evidence.  They validate the action family and the
precomputed V193 attribution, not a deployable or cross-scene policy.

