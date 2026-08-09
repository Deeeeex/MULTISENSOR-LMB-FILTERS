# Powered-GM KLA grid calibration

This controlled two-dimensional case directly integrates the normalized density power on a tensor grid. It calibrates the receiver approximation and is not a full-tracker result.

| Candidate | TV | KL oracle-to-candidate | Mean error | Cov. error | Existence error | log eta error |
|:--|--:|--:|--:|--:|--:|--:|
| frozen-powered-gm-top3-max8 | 0.033392 | 0.005930 | 0.021710 | 0.346809 | 0.004839 | +0.051653 |
| untruncated-powered-gm-3x3 | 0.033415 | 0.005939 | 0.021690 | 0.347049 | 0.004842 | +0.051684 |
| projected-single-gaussian | 0.541501 | 0.839908 | 0.061950 | 0.328971 | 0.008053 | +0.087171 |

Grid convergence (81 to 161 points per dimension):

- log eta: `2.263e-11`
- existence: `2.164e-12`
- mean: `5.808e-11`
- covariance: `2.457e-10`
- fine-grid boundary mass: `3.082e-13`
