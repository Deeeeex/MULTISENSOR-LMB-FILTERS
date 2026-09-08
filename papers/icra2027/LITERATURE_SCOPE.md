# Verified scope and remaining reproduction limits

Checked primary sources on 2026-09-08. Bibliographic records are retrieved
from Crossref, author-supplied BibTeX or official documentation and saved in
`literature/verification.json`; manuscript keys must match titles, not just
a successful HTTP response.

- Moore and Stouch, Intelligent Autonomous Systems 13, pp. 335--348,
  DOI 10.1007/978-3-319-08338-4_25: generalized EKF sensor fusion in ROS
  robot_localization. The Springer chapter abstract and author names were
  checked. Its recommended proceedings citation uses 2016; Crossref uses
  online-first 2015, so the bibliography follows the publisher's 2016 citation.
- Shan et al., LIO-SAM, IROS 2020, pp. 5135--5142: factor-graph lidar-inertial
  smoothing and mapping. Crossref DOI 10.1109/IROS45743.2020.9341176,
  author-supplied BibTeX and arXiv:2007.00258 were cross-checked. The final
  entry uses Crossref's canonical author list, including Daniela Rus.
  A partially successful batch returned the record despite other requests
  receiving HTTP 429; raw records from both sources are retained.
- Autoware Foundation, Multi Object Tracker: the official Universe module
  documentation confirms data association and EKF tracking. This is an
  online-documentation reference with access date 2026-09-08, not a journal
  paper or a performance comparison with ER.
- Gao et al., Robotics and Autonomous Systems 198 (2026), 105358,
  DOI 10.1016/j.robot.2026.105358: distributed multi-robot SLAM with LMB
  landmark maps and map fusion under missed detections and clutter. The
  publisher and University of Florence records report simulated and real
  data. This supports a robotics application of LMB, not industry adoption
  or hardware validation of the present target-tracking method.

- Wang et al., Signal Processing 2018, DOI 10.1016/j.sigpro.2018.04.010:
  information-divergence weighting for multiple-view LMB fusion predates
  this work. Do not claim first per-Bernoulli informativeness weighting.
  Primary publisher abstract plus Gao et al. 2019 introduction checked.
- Gao et al., TSP 2020, DOI 10.1109/TSP.2020.3028496; author manuscript
  https://arxiv.org/html/1911.01083v1, Proposition 3 and Sections IV/V-C:
  exact constrained LMB MIL and label-subspace fusion. `mil` implements
  zero-extension pooling; `mil_support` implements its represented-label
  subspace specialization with known common labels. Neither is a complete
  end-to-end reproduction with independent-label association.
- Gao et al., TAES 2022, DOI 10.1109/TAES.2022.3182642: publisher abstract
  confirms FoV label decomposition, constrained MIL, and label assignment.
  Do not represent the general unequal-FoV problem as unsolved.
- Li et al., Signal Processing 2021, DOI 10.1016/j.sigpro.2021.108210:
  publisher abstract explicitly recognizes useful out-of-current-FoV
  information from history and relays. Our distinction is a lightweight
  direct-opportunity age rule on LMB existence, not first historical FoV use.
- Uney et al., TAES 2019, DOI 10.1109/TAES.2019.2893083, author PDF
  https://discovery.ucl.ac.uk/id/eprint/10069137/1/08613927.pdf: cardinality
  consistency and separate cardinality/localization fusion predate this
  work. Our two weight vectors retain the spatial overlap penalty and do
  not establish that paper's cardinality consistency.
- Jin et al., DSP 2024, DOI 10.1016/j.dsp.2024.104585: publisher abstract and
  introduction checked. Shared-prior conditions and a consensus information
  selector address label sensitivity and different FoVs; do not claim that
  shared birth labels solve independent birth/label matching.
- Dames, Autonomous Robots, DOI 10.1007/s10514-019-09840-9; Ramachandran
  et al., TCNS 2021, DOI 10.1109/TCNS.2021.3059794; Banerjee and Schneider,
  ICRA 2024, DOI 10.1109/ICRA57147.2024.10609977: distributed search/tracking,
  resource-aware reconfiguration, and active search with unreliable/asynchronous
  communication provide the robotics context. Our prescribed paths do not
  compete with their action planners or solve navigation.
- Lang et al., public SSRN preprint, abstract 7129254, posted 2026-07-16:
  communication-aware adaptive KLA/LMB weighting is overlapping prior work
  and must be described in third person if cited. This manuscript reuses
  LMB/KLA code; it does not claim the estimator core as a new contribution.

The paper is an evidence-bounded simulation study with a modest fusion rule.
It has no hardware validation, raw LiDAR/image perception, localization error,
arbitrary label matching, universal calibration or consensus convergence
result. Neither the v2 nor v3 combinations passed their balanced development
gate. The simpler v1 Age-all arm passed its initial direction screen, but
lost to the lineage control and worsened common-target localization.
Preserving those facts is necessary even if validation yields conditional gains.
