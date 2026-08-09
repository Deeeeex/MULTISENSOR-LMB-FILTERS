# V63: observable X36 control-plane/data-plane schedules

## Decision being tested

V62 established a strong M24 positive control without changing the physical
carrier graph: temporarily preventing selected formations from consuming
complete cross-formation posteriors for three steps improved mean tracking by
`10.393%` while retaining nonnegative tail, consensus, and attempted-byte
metrics.  Passing labels with current measurement support weakened that gain,
so the remaining decision is temporal: **which formations should be protected,
for how long, and when should their cross-formation input return?**

V63 tests whether this mechanism has useful X36 headroom.  It is not a new
topology family and does not enumerate arbitrary schedules.  All arms use the
same registered fixed counter-clockwise physical graph and fusion weights.
Only the data plane changes: at a scheduled step, the listed formations still
exchange control synopses but do not admit complete cross-formation posteriors
into fusion.

## Observable ranking

At the opened current state, each formation is scored by the amount of
reference-relative existence loss that would be rescued by withholding its
cross-formation input, weighted by the receiving labels' current measurement
association support.  The score uses current local posteriors, current link
reliability, current positions, and two past selected topology pages.  It does
not use target truth, future measurements, future links, or future tracking
outcomes.

## Frozen seven-arm bank

The ranking is converted into a small, fixed H=3 bank before truth scoring:

1. registered reference with complete payloads;
2. the historical V37 schedule for the same anchor;
3. current top-1 formation protected for all three steps;
4. current top-2 formations protected for all three steps;
5. current top-3 formations protected for all three steps;
6. nested recovery `top-3 -> top-2 -> top-1`;
7. faster recovery `top-3 -> top-2 -> none`.

This bank separates three hypotheses with only six nonreference arms:

- scale dilution requires protecting more than one of six formations;
- X36 requires persistence rather than a one-step intervention;
- the correct recovery rate differs from the historical V37 controller.

The first outcome state is the historically weak `x36-formation-fov`, seed
`211`, `t=72` window.  Other anchors remain closed until this state either
shows a strict candidate with at least `5%` mean tracking gain or provides a
clear mechanism-level reason to revise the bank.

## Strict development gate

A nonreference arm counts only when all of the following are nonnegative
relative to the paired reference over H=3:

- worst-sensor tracking gain;
- minimum per-formation tracking gain;
- window consensus gain;
- terminal consensus gain;
- attempted posterior-byte saving;
- rolling sensor- and formation-level B3 connectivity.

Among such arms, mean tracking gain must reach `5%` to open the remaining X36
anchors.  This is opened seed-211 development evidence only; it cannot support
validation, generalization, or model-training claims.
