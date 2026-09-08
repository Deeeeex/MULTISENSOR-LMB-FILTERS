# Primary sources and experimental scope

Primary bibliographic metadata and available author manuscripts were checked
on 2026-09-08. `literature/verification.json` stores the records and retrieval
sources; raw new BibTeX records are retained alongside it. A resolved key
checks bibliographic identity, not implementation equivalence.

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
