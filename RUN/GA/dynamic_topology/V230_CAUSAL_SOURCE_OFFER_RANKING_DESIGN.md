# V230 causal source-offer ranking

## Why this is the next gate

The beneficiary-only V229 query rule misses both opened t=133 teacher labels
because it cannot observe remote surprise before choosing three label keys.
The recursive filter already maintains the required source-local state:
`receivedCache{receiver,sender}` is updated only after a message is actually
delivered.  A source can therefore compare its current label posterior with
`receivedCache{source,beneficiary sensor}` without reading a global posterior
snapshot.  The relevant age is the last successful delivery in the reverse
direction, from beneficiary sensor to source.

Existing V218/V226 continuation caches do **not** persist this runtime cache.
They store one global pre-topology local-posterior page and two delivered graph
pages.  Those files may establish which reverse edges recently delivered, but
they cannot reconstruct the actual cached payload and must not be treated as
source-local evidence.

## Frozen two-offer rule

Every source may return at most two records under the already charged V229
message contract.  Only full ordinary cache entries no more than two pages old
are usable; a missing label counts as negative evidence only in such a full
entry.

1. **Existence surprise:** rank the absolute source-to-cache log-odds gap,
   weighted by the larger existence probability.  This covers both a strong
   positive source label absent from the beneficiary and a credible negative
   source that can suppress a false or stale label.
2. **Spatial utility:** rank covariance-normalized position disagreement plus
   a one-sided precision advantage when the source is more precise than the
   cached beneficiary state.

One label is taken from each mode.  Exact ties prefer the smaller complete
payload and use numeric label values only as deterministic final tie-breaks.
The ranker does not use truth, future outcomes, or labels held by any node
outside the source's own causal cache.

## Evidence sequence

The first screen is deliberately optimistic: substitute every beneficiary's
current full posterior for a perfectly fresh cache and test whether the two
teacher source-label rows enter the two offers.  Failure closes this score
without another tracking run.  Success authorizes only the following work:

1. persist the true `receivedCache`, its event type, and reverse-link age at
   selected capture pages;
2. repeat recall using only cache entries actually owned by each source;
3. execute the selected complete label through the existing V226 eta/KLA
   projection and exact V229 byte ledger;
4. only after causal recall and material teacher headroom both pass, compare a
   frozen online source-offer rule with the V227 static and dynamic references.

The same-state proxy is not an online result and cannot establish tracking,
communication, or generalization gain.
