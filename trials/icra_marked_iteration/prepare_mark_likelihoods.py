"""Turn previously frozen cross-fitted probabilities into likelihood ratios."""
from pathlib import Path
import csv,hashlib,json
import numpy as np
from scipy.io import loadmat,savemat
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
PRIOR=OUT.parent/'icra_ceiling_iteration'


def main():
    calibration=json.loads((PRIOR/'calibration.json').read_text())
    path=OUT.parent/'icra_evidence_iteration/detection_score_diagnosis.csv'
    assert hashlib.sha256(path.read_bytes()).hexdigest()==calibration['diagnosis_input_sha256']
    with path.open() as f:rows=list(csv.DictReader(f))
    seq=np.array([r['sequence'] for r in rows]);y=np.array([int(r['positive_12m']) for r in rows])
    priors={s:float(y[seq==s].mean()) for s in sorted(set(seq))};records=[]
    dest=OUT/'data_likelihoods';dest.mkdir(exist_ok=True)
    for fold,entry in zip(calibration['folds'],calibration['inputs']):
        name=fold['excluded_sequence'];assert name==entry['sequence']
        selected=[s for s in priors if s!=name];assert selected==fold['training_sequences']
        pi=float(np.mean([priors[s] for s in selected]));assert 0<pi<1
        src=PRIOR/'data_marks'/f'marks_{name}.mat';assert hashlib.sha256(src.read_bytes()).hexdigest()==entry['marks_sha256']
        marks=loadmat(src);probability=marks['calibratedScores'];ratios=np.empty(probability.shape,object)
        for index in np.ndindex(probability.shape):
            p=np.clip(probability[index],1e-6,1-1e-6)
            ratios[index]=p/(1-p)*(1-pi)/pi
            assert np.isfinite(ratios[index]).all() and (ratios[index]>0).all()
        target=dest/f'likelihoods_{name}.mat'
        savemat(target,dict(likelihoodRatios=ratios),do_compression=True)
        records.append(dict(sequence=name,training_sequences=selected,positive_prior=pi,
                            likelihoods_sha256=hashlib.sha256(target.read_bytes()).hexdigest()))
    result=dict(protocol='marked-evidence-ceiling-v1',calibration_sha256=hashlib.sha256((PRIOR/'calibration.json').read_bytes()).hexdigest(),
                per_sequence_positive_fraction=priors,full_seen_positive_prior=float(np.mean(list(priors.values()))),inputs=records)
    (OUT/'likelihood_manifest.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('Fixed marked likelihoods prepared; full-seen sequence-balanced positive prior',result['full_seen_positive_prior'])


if __name__=='__main__':main()
