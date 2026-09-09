"""Controlled correlated Gaussian observations: error and uncertainty coverage.

The conditional-existing-object spatial special case of unit-admission GCE.
No estimated score gates, real detections, tracking recursion, or new-data
generalization claim is involved. Oracle uses the true joint noise covariance.
"""
from pathlib import Path
import hashlib
import json
import csv

import numpy as np
from scipy.stats import chi2

OUT=Path(__file__).resolve().parent


def main():
    spec=dict(seed=20260909,samples_per_rho=10000,rhos=[0,.25,.5,.75,.9],
              dimensions=2,prior_mean=[0,0],prior_variance=25.,measurement_variance=1.,
              assumed_cross_source_correlation=0.,nominal_region_probability=.95,
              arms=['No-age spatial pool','Unit-admission GCE','Known-correlation oracle'],
              code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    freeze=OUT/'CORRELATION_FREEZE.json'
    assert not freeze.exists() and not (OUT/'correlation_control.json').exists()
    freeze.write_text(json.dumps(spec,indent=2)+'\n')
    rows=[];arrays={};N=spec['samples_per_rho'];D=spec['dimensions'];P0=spec['prior_variance']
    q=chi2.ppf(spec['nominal_region_probability'],D)
    children=np.random.SeedSequence(spec['seed']).spawn(len(spec['rhos']))
    for rho,seed in zip(spec['rhos'],children):
        rng=np.random.default_rng(seed)
        truth=rng.normal(size=(N,D))*np.sqrt(P0)
        common=rng.normal(size=(N,D));noise=rng.normal(size=(N,2,D))
        eps=np.sqrt(rho)*common[:,None,:]+np.sqrt(1-rho)*noise
        observations=truth[:,None,:]+eps;average=observations.mean(1)
        Ppool=1/(1/P0+1);Pgce=1/(1/P0+2);Poracle=1/(1/P0+2/(1+rho))
        settings=[(spec['arms'][0],Ppool,Ppool),
                  (spec['arms'][1],Pgce,2*Pgce),
                  (spec['arms'][2],Poracle,2*Poracle/(1+rho))]
        sample={}
        for name,P,gain in settings:
            mean=gain*average;error=mean-truth;squared=(error**2).sum(1);nees=squared/P
            covered=nees<=q;coverage=float(covered.mean())
            # Exact prior-predictive error variance for each estimator.
            error_variance=(gain-1)**2*P0+gain**2*(1+rho)/2
            expected_coverage=float(chi2.cdf(q*P/error_variance,D))
            half=1.96*np.sqrt(coverage*(1-coverage)/N+1.96**2/(4*N**2))/(1+1.96**2/N)
            center=(coverage+1.96**2/(2*N))/(1+1.96**2/N)
            row=dict(rho=rho,arm=name,samples=N,position_rmse=float(np.sqrt(squared.mean())),
                     expected_rmse=float(np.sqrt(D*error_variance)),reported_position_variance=P,
                     mean_nees_per_dimension=float(nees.mean()/D),expected_nees_per_dimension=error_variance/P,
                     coverage_95=coverage,expected_coverage_95=expected_coverage,
                     coverage_wilson_lower=center-half,coverage_wilson_upper=center+half,
                     admitted_source_fraction=1. if name==spec['arms'][1] else None)
            assert np.isfinite(squared).all() and P>0
            rows.append(row);sample[name]=(mean,P)
            arrays[f'rho_{rho}_{name.replace(" ","_")}']=np.c_[squared,nees,covered]
        if rho==0:
            assert np.array_equal(sample[spec['arms'][1]][0],sample[spec['arms'][2]][0])
            assert sample[spec['arms'][1]][1]==sample[spec['arms'][2]][1]
        # Every local update has delta J=I, irrespective of cross correlation.
        assert np.linalg.eigvalsh(np.eye(D)).min()==1
    sample_path=OUT/'correlation_samples.npz';np.savez_compressed(sample_path,**arrays)
    result=dict(specification=spec,rows=rows,
                freeze_sha256=hashlib.sha256(freeze.read_bytes()).hexdigest(),
                samples_sha256=hashlib.sha256(sample_path.read_bytes()).hexdigest(),
                zero_correlation_gce_oracle_exact=True,
                scope='Prior-predictive Gaussian spatial special case; PSD does not test unknown cross-source correlation. Not full real tracker consistency.')
    (OUT/'correlation_control.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    with (OUT/'correlation_control.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    for r in rows:
        print(r['rho'],r['arm'],'RMSE',round(r['position_rmse'],3),'NEES/d',round(r['mean_nees_per_dimension'],3),
              'coverage',round(r['coverage_95'],4),'analytic coverage',round(r['expected_coverage_95'],4))


if __name__=='__main__':main()
