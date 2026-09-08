# Evidence-ceiling contribution boundary

Checked the publisher's full article and Crossref metadata on 2026-09-08:
[Sun et al., IET Radar, Sonar & Navigation 2019](https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/iet-rsn.2018.5293).
The paper uses neighboring-cell amplitude likelihood information in LMB
tracking to distinguish marine targets from clutter. Its introduction also
describes earlier amplitude-likelihood-ratio association methods. Thus adding
an observation mark or likelihood ratio to the local LMB update is established
practice, and is an information-matched control here, not a novelty claim.
The publisher and Crossref agree on all four authors, volume 13, issue 6,
pages 983--991 and DOI. The raw metadata and normalized BibTeX are adjacent.

The candidate contribution concerns how current direct-association support
limits positive changes caused by recency weights in the Bernoulli fusion
step. The scalar constrained-KL expression is an interpretation of that
specific rule. It does not establish a calibrated statistical upper bound,
consistency of cardinality, recovery of independent likelihood evidence,
unknown-correlation Bayes optimality or convergence of consensus.

The existing paper's primary-source boundaries for LMB, KLA, information
weighting, separate cardinality/localization fusion, FoV label partitioning
and trajectory consensus remain applicable. The final paper must compare
the candidate against baselines given the same score information. An
improvement over unmarked baselines alone would combine observation-model
and fusion-rule contributions. No bibliographic search can prove that a
specific clipping formula has never appeared elsewhere; avoid priority claims.
