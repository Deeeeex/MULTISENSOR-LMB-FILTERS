# Primary sources and experimental scope

Primary bibliographic metadata and available author manuscripts were checked
on 2026-09-08. `literature/verification.json` stores the records and retrieval
sources; raw new BibTeX records are retained alongside it. A resolved key
checks bibliographic identity, not implementation equivalence.

The dedicated eighth page uses full primary publication titles and prints
verified non-arXiv DOI identifiers as plain text. These identifiers come
from the same verified records; no extra references or PDF hyperlink
annotations are introduced by this formatting change.

## Expanded manuscript coverage

Additional coverage was checked on 2026-09-09. The manuscript now cites
30 distinct records, up from 18. Nine newly
retrieved records and three already verified but previously uncited
records support the following additions. Bibliographic metadata comes
from Crossref, DataCite, or the official NeurIPS/CVF proceedings; the
linked author texts and abstracts bound the claims made in the paper.

| Added citation | Primary record or text | Role in the manuscript |
| --- | --- | --- |
| Vo and Vo, TSP 2013 | [Author manuscript](https://ba-ngu.vo-au.com/vo/VV_Conjugate_TSP13.pdf), DOI 10.1109/TSP.2013.2259822 | Labeled finite-set filtering foundations. |
| Julier and Uhlmann, ACC 1997 | DOI 10.1109/ACC.1997.609105; CI background in the [Uney author manuscript](https://www.pure.ed.ac.uk/ws/files/19646890/preprint_emd_fusion.pdf) | Estimate fusion with unknown cross-correlations. The original 1997 full text was not retrieved. |
| Uney et al., JSTSP 2013 | [Author manuscript](https://www.pure.ed.ac.uk/ws/files/19646890/preprint_emd_fusion.pdf), DOI 10.1109/JSTSP.2013.2257162 | Exponential-mixture finite-set and PHD fusion. |
| Fantacci et al., arXiv 2015 | [Author preprint](https://arxiv.org/abs/1501.01579) | Consensus labeled multi-object filters. Cited as a preprint without an unverified journal assignment. |
| Li et al., TSP 2018 | [Author preprint](https://arxiv.org/abs/1710.00501), DOI 10.1109/TSP.2017.2760286 | Sensitivity of labeled GCI fusion to inconsistent labels. |
| Li et al., TSP 2019 | [Institutional author abstract](https://flore.unifi.it/handle/2158/1140832), DOI 10.1109/TSP.2018.2880704 | Linear assignment for label matching before fusion. Volume 67 follows the DOI registry rather than the repository's inconsistent volume field. |
| Li et al., Signal Processing 2020 | [Author preprint](https://arxiv.org/abs/1903.06985), DOI 10.1016/j.sigpro.2019.107246 | Clustering and compensation for differing sensing fields of view. |
| Williams and Lau, TAES 2014 | [Author preprint](https://arxiv.org/abs/1209.6299), DOI 10.1109/TAES.2014.120568 | Belief-propagation approximation of marginal association probabilities. |
| Niculescu-Mizil and Caruana, ICML 2005 | [Author manuscript](https://www.cs.cornell.edu/~alexn/papers/calibration.icml05.crc.rev3.pdf), DOI 10.1145/1102351.1102430 | Post-hoc probability calibration as background for the local score fit. |
| Banerjee and Schneider, ICRA 2024 | [Author preprint](https://arxiv.org/abs/2401.03154), DOI 10.1109/ICRA57147.2024.10609977 | Decentralized active search and tracking with PHD inference. |
| Hu et al., NeurIPS 2022 | [Official proceedings](https://proceedings.neurips.cc/paper_files/paper/2022/hash/1f5c5cd01b864d53cc5fa0a3472e152e-Abstract-Conference.html) | Where2comm's confidence-guided sparse feature communication. |
| Zhong et al., ICCV 2025 | [Official proceedings](https://openaccess.thecvf.com/content/ICCV2025/html/Zhong_CoopTrack_Exploring_End-to-End_Learning_for_Efficient_Cooperative_Sequential_Perception_ICCV_2025_paper.html) | CoopTrack's sparse instance features and learned cross-agent association. |

The robotics and feature-exchange papers establish context; they are not
additional evaluated baselines. The local association and calibration
citations support the shared backend. Related work distinguishes these
roles from GCE's current Bernoulli ratio and joint normalizer.

## Principles that precede GCE

- Reuter et al., TSP 2014, DOI 10.1109/TSP.2014.2323064: the LMB filter and
  Bernoulli representation are established components.
- Battistelli and Chisci, Automatica 2014, DOI 10.1016/j.automatica.2013.11.042:
  Kullback–Leibler averaging is the geometric fusion baseline.
- Wu et al., [Bayesian data fusion with shared priors](https://arxiv.org/abs/2212.07311):
  shared-prior correction and prior/likelihood separation are established.
  The manuscript's common-prior identity is a limiting-case consistency
  check, not a new Bayesian fusion theorem. Metadata comes from DataCite,
  DOI 10.48550/arXiv.2212.07311; the citation identifies the 2022 preprint.
- Hlinka et al., TSP 2012, DOI 10.1109/TSP.2012.2196697,
  [author manuscript](https://arxiv.org/abs/1108.6214): likelihood consensus
  aggregates exponential-family likelihood representations. Current
  natural-parameter increments are therefore not a new principle by themselves.
- Yi and Chai, TSP 2021, DOI 10.1109/TSP.2021.3087033,
  [author manuscript](https://arxiv.org/abs/2106.08088): heterogeneous
  component-level confidence in RFS fusion predates this work.
- Uney et al., TAES 2019, DOI 10.1109/TAES.2019.2893083:
  cardinality and spatial consistency can differ. GCE's separate base
  weights do not establish that paper's cardinality-consistency guarantee.

GCE's claimed contribution is the guarded, jointly normalized Bernoulli
realization for approximate local LMB updates, its exact-zero transport,
and the stated controlled experiment. Independence, universal calibration,
and general correlation removal are not established.

## Sensing support and external adaptations

- Li et al., FUSION 2018, DOI 10.23919/ICIF.2018.8455250: multi-object LMB
  fusion with different fields of view is existing work. Absent-label and
  exclusive-information handling are not first introduced here.
- Gao et al., TSP 2020, DOI 10.1109/TSP.2020.3028496, and
  [public author manuscript, version 1](https://arxiv.org/html/1911.01083v1):
  MIL arithmetic pooling and common/exclusive label subspaces. The replay
  implements the public manuscript's augmented label assignment with
  Gaussian symmetric-KL costs and moment-projected spatial mixtures.
  The preprint version is cited separately instead of silently equating
  its algorithm to every later published version.
- Gao et al., TAES 2022, DOI 10.1109/TAES.2022.3182642: differing-FoV MIL
  fusion and label assignment. Full equivalence to that implementation
  and original experimental protocol remains unverified.
- Nguyen et al., TSP 2021, DOI 10.1109/TSP.2021.3103125,
  [author manuscript](https://arxiv.org/abs/2012.12990) and public
  AdelaideAuto-IDLab MATLAB code: author kinematic track matching and
  two-stage fusion functions are used with windows five and ten. Adapter
  outputs were checked against the author entry point. Independent local
  labels and no density feedback are retained. Set metrics do not verify
  network-wide identity consensus.

These methods share the local observation model and delivery opportunities
in this replay, but their original architectures and benchmarks differ.
The paper does not claim a reproduction of their native protocols.

## Data, robotics setting, and metrics

- Xu et al., V2V4Real, CVPR 2023, DOI 10.1109/CVPR52729.2023.01318;
  Chiu et al., DMSTrack, ICRA 2024, DOI 10.1109/ICRA57147.2024.10610487:
  metadata and author-released per-vehicle detections, scores, relative
  transforms, and annotations were checked. The current experiment uses
  nine validation and twenty-five train sequences, all already seen during
  fusion development. The train detections are from the detector training
  split. It does not reproduce DMSTrack's learned filtering or official
  three-dimensional benchmark scores.
- Dames, Autonomous Robots, DOI 10.1007/s10514-019-09840-9; Ramachandran
  et al., TCNS 2021, DOI 10.1109/TCNS.2021.3059794; Gao et al., RAS 2026,
  DOI 10.1016/j.robot.2026.105358: distributed search, sensing-resource
  reconfiguration, and LMB robot landmark mapping provide robotics context.
  They are not evidence for real robot execution or planning by GCE.
- Schuhmacher et al., TSP 2008, DOI 10.1109/TSP.2008.920469, and Rahmathullah
  et al., FUSION 2017, DOI 10.23919/ICIF.2017.8009645: OSPA and GOSPA
  definitions. Missed and false GOSPA components are reported as squared
  costs, with the fixed cutoff and order stated in the experiment section.

No old synthetic recency result is used as evidence for the new GCE rule.
The portable package retains the complete snapshots behind the current
paper; archived experiment files remain in the versioned repository.
