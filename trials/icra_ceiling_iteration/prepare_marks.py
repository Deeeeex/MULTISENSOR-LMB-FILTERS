"""Fixed, sequence-held-out score calibration; no tracking outcomes used."""
from pathlib import Path
import csv, hashlib, json
import numpy as np
from scipy.io import loadmat, savemat
from scipy.optimize import minimize
from scipy.special import expit

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
DIAG=OUT.parent/'icra_evidence_iteration/detection_score_diagnosis.csv'
ORIGINAL=OUT.parent/'icra_external_fusion'


def fit(x,y,seq,train):
    w=np.zeros(len(x))
    for s in sorted(set(seq[train])):
        mask=train&(seq==s);w[mask]=1/mask.sum()
    w/=w.sum()
    def objective(theta):
        a,b=theta;z=a*x+b
        p=expit(z);loss=np.sum(w*(np.logaddexp(0,z)-y*z))+.001*a*a
        residual=w*(p-y)
        grad=np.array([np.sum(residual*x)+.002*a,np.sum(residual)])
        return loss,grad
    result=minimize(objective,[1.,0.],jac=True,method='L-BFGS-B',bounds=[(0,None),(None,None)],
                    options=dict(ftol=1e-13,gtol=1e-9,maxiter=1000))
    assert result.success,(result.message,result.x)
    return dict(a=float(result.x[0]),b=float(result.x[1]),training_sequences=sorted(set(seq[train])),
                train_n=int(train.sum()),objective=float(result.fun),iterations=int(result.nit))


def metrics(y,p):
    p=np.clip(p,1e-12,1-1e-12)
    return dict(n=len(y),brier=float(np.mean((p-y)**2)),
                log_loss=float(np.mean(-y*np.log(p)-(1-y)*np.log1p(-p))))


def main():
    with DIAG.open() as f:rows=list(csv.DictReader(f))
    seq=np.array([r['sequence'] for r in rows]);sensor=np.array([r['sensor'] for r in rows])
    frame=np.array([int(r['frame']) for r in rows]);raw=np.array([float(r['score']) for r in rows])
    y=np.array([int(r['positive_12m']) for r in rows]);x=np.log(np.clip(raw,1e-6,1-1e-6))-np.log1p(-np.clip(raw,1e-6,1-1e-6))
    assert ((raw>=0)&(raw<=1)).all()
    folds=[];calibrated=np.zeros(len(rows));inputs=[];(OUT/'data_marks').mkdir(exist_ok=True)
    for name in sorted(set(seq)):
        excluded=seq==name;model=fit(x,y,seq,~excluded)
        calibrated[excluded]=expit(model['a']*x[excluded]+model['b'])
        folds.append(dict(excluded_sequence=name,**model,raw=metrics(y[excluded],raw[excluded]),
                          calibrated=metrics(y[excluded],calibrated[excluded])))
        source=ORIGINAL/'data'/f'v2v4real_{name}.mat';data=loadmat(source);T=int(data['T'].item())
        marks=np.empty((2,T),dtype=object);cal=np.empty((2,T),dtype=object)
        for n,s in enumerate(['ego','1']):
            for t in range(T):
                mask=excluded&(sensor==s)&(frame==t)
                assert mask.sum()==data['measurements'][n,t].shape[1]
                marks[n,t]=raw[mask].reshape(1,-1);cal[n,t]=calibrated[mask].reshape(1,-1)
        path=OUT/'data_marks'/f'marks_{name}.mat'
        savemat(path,dict(rawScores=marks,calibratedScores=cal),do_compression=True)
        inputs.append(dict(sequence=name,original_input_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                           marks_sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    full=fit(x,y,seq,np.ones(len(rows),bool))
    result=dict(protocol='direct-evidence-ceiling-v1',label='per-sensor assignment within 12 m',
                diagnosis_input_sha256=hashlib.sha256(DIAG.read_bytes()).hexdigest(),folds=folds,
                full_seen_fit=full,raw=metrics(y,raw),held_out=metrics(y,calibrated),inputs=inputs)
    (OUT/'calibration.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(raw=result['raw'],held_out=result['held_out'],full_seen_fit=full),indent=2))
    for fold in folds:print(fold['excluded_sequence'],fold['a'],fold['b'],fold['calibrated'])


if __name__=='__main__':main()
