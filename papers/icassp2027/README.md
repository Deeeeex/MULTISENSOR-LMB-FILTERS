# Independent ICASSP 2027 manuscript

This directory starts a new paper from the official 2027 template archive.
It does not reuse the previous ICASSP manuscript or the Adaptive-KLA text.

- `main.tex` / `main.pdf`: English research draft.
- `MAINLINE_PROGRESS_CN.md`: current direction, paired result tables and next decision.
- `references.bib`: metadata retrieved from DOI/Crossref records.
- `TEMPLATE_SOURCE.md`: template origin and official requirements.
- `official-template/`: untouched style sources and the complete downloaded archive.

Build from this directory:

```sh
tectonic --keep-logs main.tex
```

The working title is **Delivery-Aware Sparse Causal Routing for Distributed
Multi-Object Tracking**. Jinhao Chen is first author; Tianyu Wo is corresponding
author. Both affiliations are Beihang University, as confirmed by the author.
No funding and no conflicts of interest were declared.
The supplied English names are authoritative. Chinese-character expansions
in the message of commit `1185a5d` were not author-confirmed and should not
be used; they do not appear in the manuscript or its author block.

The canonical progress document remains
[the existing Lark document](https://jjp48fb03jzs.jp.larksuite.com/docx/HcFFdtKIRovhHLxKrx5jVpiBpJh).
The independent paper is maintained on `codex/icassp2027-sparse-causal-routing`.

The manuscript contains the implemented sparse causal backbone, a scheduled
connectivity/count proof, a conditional finite-round geometric-pooling bound,
and complete three-arm M24/X36 development results. The X36 self-weight
fallback ablation is also complete: set error and consistency improve over
sparse renormalization, but conditional RMSE worsens by 4.836%, failing the
predefined M24 follow-up gate. The draft now includes the V279 count-error
budget, common-finite-cell RMSE comparison and a compact V280 opportunity
readout. Two Python-rendered vector figures replace the original stacked
pipeline boxes and expose the payload/set-error tradeoff. Figure sources,
data and regeneration instructions are in `figures/README.md`.
No multiseed significance, exact arbitrary-GM powers, globally minimum graph,
loss-resilient equal-weight consensus or end-to-end bandwidth saving is claimed.

Before submission: locate the target-existence bottleneck and establish a
jointly beneficial method on both scales; freeze independent seeds and the
second scene style; assess control traffic;
expand the routing-specific related-work comparison and establish novelty;
finalize the author-reviewed manuscript and any required AI-use disclosure.
The derived perturbation bound is elementary and scoped; it is not by itself
evidence of a novel estimation-optimal policy.

V280 remains a geometry/packet-opportunity diagnostic, not an actual-label
propagation experiment; its headline readout now appears in the manuscript.
It reuses opened cases and changes no filter or sensing parameter. The V278
background process finished normally; no automatic M24 extension was started.

The routing-related comparison now includes Ramachandran et al., IEEE TCNS
2021, DOI 10.1109/TCNS.2021.3059794. Topology reconfiguration, small edit count
and joint topology/weight design already have prior art. Our remaining
novelty burden is the useful interaction of a fixed sparse budget, directed
delivery and finite-round LMB target-set recovery, not dynamic topology alone.
