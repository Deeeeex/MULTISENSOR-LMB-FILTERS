# V131 finding: intermittent whole-network refresh is informative but unsafe

V131 does not pass the registered X36 development gate and is retained only as
an experiment record. It must not be promoted to the canonical progress
document.

## Paired result

Against the matched static full-payload reference on X36 seed 211, `t=72`,
`H=8`:

- mean E-OSPA gain: `+6.180%`;
- post-maturity minimum gain: `+4.426%` (below the `+5%` gate);
- formation gains: `[+0.824, +7.169, +7.711, +8.970, +11.250, +1.244]%`;
- minimum formation-time gain: `-9.649%`;
- F6 non-gateway terminal gain: `+3.149%`;
- window / terminal consensus gains: `+12.628% / +29.358%`;
- attempted-byte saving after charging the anchor: `+6.465%`;
- auxiliary light traffic: `240` messages and `1,051,200` attempted bytes;
- candidate runtime: `268.03 s` versus `251.51 s` for the reused static arm.

## Mechanism diagnosis

The light network anchor is much stronger than a zero-message local anchor:
it turns F6's terminal peer gain positive and improves the aggregate result.
However, refreshing the entire network only on alternating pages leaves the
actual rollback recipients under-served at the pages where their state is
used. F1 remains negative on pages 6--8 (`-2.150%, -4.837%, -7.575%`), while
F2 suffers a `-9.649%` shock on page 5. This is a temporal allocation failure,
not a lack-of-bandwidth failure: the same 240-message budget can instead
refresh only the F1/F2/F6 receiver rows on every page.

## Next decision

Keep the working V105 path unchanged and reallocate the same auxiliary message
budget from whole-network alternating refreshes to risk-formation continuous
refreshes. The next mechanism test must still charge actual bytes, preserve
positive net byte saving, and use the same strict formation-time gate. If that
reallocation also fails, close the parallel compressed-anchor route rather
than adding more privileged patches.
