"""Recording-held-out recall calibration, without tracking output inputs."""
from pathlib import Path
import csv
import hashlib
import io
import json

import numpy as np
from scipy.io import loadmat
from scipy.optimize import linear_sum_assignment, minimize
from scipy.special import expit

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()


def targets_matched(truth,measurements,cutoff):
    squared=((truth[:,None,:]-measurements[None,:,:])**2).sum(axis=2)
    i,j=linear_sum_assignment(np.minimum(squared,cutoff**2))
    matched=np.zeros(len(truth),bool)
    matched[i[squared[i,j]<cutoff**2]]=True
    return matched


def weights(sequences,mask):
    result=np.zeros(len(sequences))
    for sequence in sorted(set(sequences[mask])):
        part=mask&(sequences==sequence)
        result[part]=1/part.sum()
    return result/result.sum()


def fit(x,y,sequences,train):
    w=weights(sequences,train)
    constant=float(np.dot(w,y))
    def objective(theta):
        a,b=theta
        z=a+b*x
        p=expit(z)
        loss=np.sum(w*(np.logaddexp(0,z)-y*z))+.001*b*b
        residual=w*(p-y)
        return float(loss),np.array([residual.sum(),np.sum(residual*x)+.002*b])
    result=minimize(objective,[np.log(constant/(1-constant)),0.],jac=True,method='L-BFGS-B',
                    bounds=[(None,None),(None,0)],options=dict(ftol=1e-13,gtol=1e-9,maxiter=1000))
    assert result.success,(result.message,result.x)
    assert np.linalg.norm(objective(result.x)[1],ord=np.inf)<1e-7 or result.x[1]==0
    return dict(intercept=float(result.x[0]),slope=float(result.x[1]),constant=constant,
                training_sequences=sorted(set(sequences[train])),train_rows=int(train.sum()),
                objective=float(result.fun),iterations=int(result.nit))


def metrics(y,p):
    p=np.clip(np.broadcast_to(p,np.shape(y)),1e-12,1-1e-12)
    return dict(log_loss=float(np.mean(-y*np.log(p)-(1-y)*np.log1p(-p))),
                brier=float(np.mean((p-y)**2)))


def main():
    inventory_path=OUT.parent/'icra_full_coverage/DATA_INVENTORY.json'
    inventory=json.loads(inventory_path.read_text())
    manifests=OUT.parent/'icra_external_fusion/v2v4real_input_manifest.json'
    manifest=json.loads(manifests.read_text())
    inputs={str(path.relative_to(ROOT)):sha(path) for path in [inventory_path,manifests,OUT/'RANGE_RECALL_PROTOCOL.md']}
    sequences=[row for row in inventory['rows'] if row['split']=='test']
    assert len(sequences)==9
    rows=[]
    for sequence in sequences:
        name=sequence['sequence']
        path=OUT.parent/'icra_external_fusion/data'/f'v2v4real_{name}.mat'
        expected=next(row['input_sha256'] for row in manifest['sequences'] if row['sequence']==name)
        assert sha(path)==expected
        inputs[str(path.relative_to(ROOT))]=expected
        data=loadmat(path);T=int(data['T'].item())
        for frame in range(T):
            truth=data['truth'][0,frame][:2,:].T
            ids=data['truthIds'][0,frame].reshape(-1)
            for source in range(2):
                origin=data['positions'][:,source,frame]
                distance=np.linalg.norm(truth-origin,axis=1)
                support=distance<=40
                values=truth[support]
                measurements=data['measurements'][source,frame].T
                m2=targets_matched(values,measurements,2.)
                m12=targets_matched(values,measurements,12.)
                for target,d,a,b in zip(ids[support],distance[support],m2,m12):
                    rows.append(dict(sequence=name,recording=sequence['recording'],frame=frame+1,
                                     source=source+1,target_id=int(target),range_m=float(d),matched_2m=int(a),matched_12m=int(b)))
    names=np.array([row['sequence'] for row in rows])
    recordings=np.array([row['recording'] for row in rows])
    x=np.array([row['range_m']/40 for row in rows])
    y=np.array([row['matched_2m'] for row in rows],float)
    y12=np.array([row['matched_12m'] for row in rows],float)
    models={};predictions=np.full(len(rows),np.nan);constants=np.full(len(rows),np.nan)
    for recording in sorted(set(recordings)):
        test=recordings==recording
        model=fit(x,y,names,~test)
        assert not set(names[test])&set(model['training_sequences'])
        models[recording]=model
        predictions[test]=expit(model['intercept']+model['slope']*x[test])
        constants[test]=model['constant']
    assert np.isfinite(predictions).all() and np.isfinite(constants).all()
    summary=[]
    for name in sorted(set(names)):
        mask=names==name
        summary.append(dict(sequence=name,recording=str(recordings[mask][0]),rows=int(mask.sum()),
                            recall_2m=float(y[mask].mean()),recall_12m=float(y12[mask].mean()),
                            nominal=metrics(y[mask],.9),constant=metrics(y[mask],constants[mask]),
                            range=metrics(y[mask],predictions[mask])))
    macro={arm:{key:float(np.mean([row[arm][key] for row in summary])) for key in ['log_loss','brier']} for arm in ['nominal','constant','range']}
    full=fit(x,y,names,np.ones(len(y),bool))
    full['probabilities_by_range']={str(r):float(expit(full['intercept']+full['slope']*r/40)) for r in [5,10,20,30,39]}
    bins=[]
    for low,high in [(0,10),(10,20),(20,30),(30,40)]:
        mask=(x*40>=low)&(x*40<high+(1e-10 if high==40 else 0))
        bins.append(dict(range_m=[low,high],rows=int(mask.sum()),recall_2m=float(y[mask].mean()),recall_12m=float(y12[mask].mean())))
    destination=OUT/'RANGE_RECALL_ROWS.csv';assert not destination.exists()
    buffer=io.StringIO();writer=csv.DictWriter(buffer,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    destination.write_text(buffer.getvalue())
    answer=dict(passed=True,rows=len(rows),sequences=9,recordings=len(models),folds=models,sequence_metrics=summary,
                macro=macro,full_fit=full,descriptive_range_bins=bins,
                calibration_gate_passed=macro['range']['log_loss']<macro['constant']['log_loss'],
                inputs=inputs,rows_sha256=sha(destination),source_sha256=sha(Path(__file__)),
                scope='All nine exposed V2V development sequences; entire recording excluded per fold; no V2X fit or tracking outcomes.')
    destination=OUT/'RANGE_RECALL_CALIBRATION.json';assert not destination.exists()
    destination.write_text(json.dumps(answer,indent=2,allow_nan=False)+'\n')
    print('RANGE RECALL CALIBRATION',json.dumps({key:answer[key] for key in ['rows','recordings','macro','full_fit','descriptive_range_bins','calibration_gate_passed']}),flush=True)


if __name__=='__main__':main()
