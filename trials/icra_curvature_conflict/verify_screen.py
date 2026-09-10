"""Independent density algebra, extraction, assignment scores and decision audit.

The producer builds natural parameters from local prior/posterior moments.
This verifier starts with the saved normalized fused density, removes the
old transmitted residual (GCE only), and adds the new encoded residual.
It uses a separate cardinality recurrence and score implementation.
"""
from pathlib import Path
import csv
import gzip
import hashlib
import json
import math
import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.special import expit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
RULES = ['original','paired_veto','positive_rejected_veto','negative_rejected_veto']
METRICS = ['ospa','gospa','loc2','miss2','false2','countError','outputCount']
LOWER = tuple(np.asarray(v) for v in zip(*[(r,c) for c in range(4) for r in range(c,4)]))
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def matrix(v):
    out = np.zeros((*v.shape[:-1],4,4))
    out[...,LOWER[0],LOWER[1]] = v; out[...,LOWER[1],LOWER[0]] = v
    return out


def rebuilt(run, rule, scalar):
    rec = np.asarray(run['iterationRecords'],float).reshape(-1,60)
    packets = np.asarray(run['packetGaussianRecords'],float).reshape(-1,19)
    local = np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
    gauss = np.asarray(run['localGaussianRecords'],float).reshape(-1,32)
    packet_map = {tuple(v[:4].astype(int)):v[4:] for v in packets}
    local_map = {tuple(v[:4].astype(int)):v for v in local}
    cov_map = {tuple(v[:4].astype(int)):matrix(v[22:32]) for v in gauss}
    original = rec[:,31:35].reshape(-1,2,2).astype(int); present = original[:,:,0] > 0
    encoded = np.zeros((len(rec),2,15)); support = np.zeros((len(rec),2))
    good = np.ones(present.shape,bool)
    for i,s in np.argwhere(present):
        t,n = map(int,rec[i,:2]); key = (t,n if s == 0 else 3-n,*original[i,s])
        encoded[i,s] = packet_map[key]; row = local_map[key]
        if rec[i,21+s] == t:
            support[i,s] = row[8]
        posterior_j = np.linalg.solve(cov_map[key],np.eye(4)); posterior_j = (posterior_j+posterior_j.T)/2
        increment_j = matrix(encoded[i,s,:10]); prior_j = posterior_j-increment_j
        tolerance = 1e-10*max(1.,np.linalg.norm(posterior_j,2),np.linalg.norm(prior_j,2))
        good[i,s] = np.linalg.eigvalsh(increment_j)[0] >= -tolerance
    active = rec[:,13:15] > 0; joint = (active.sum(1)>=2) & ((~active)|present).all(1)
    raw = joint[:,None]*(active-rec[:,28:30])*np.where(rec[:,19:21]>=0,support,rec[:,35:37])
    called = (raw>0).any(1); allowed = (~called[:,None]) | (~present) | good; kept = raw*allowed
    mean_actual = np.c_[rec[:,4:6],rec[:,40:42]]; covariance_actual = matrix(rec[:,42:52])
    root = np.linalg.cholesky(covariance_actual)
    actual_j = np.linalg.solve(covariance_actual,np.broadcast_to(np.eye(4),covariance_actual.shape))
    actual_j = (actual_j+actual_j.swapaxes(-1,-2))/2
    actual_h = np.einsum('nij,nj->ni',actual_j,mean_actual)
    constant = -.5*(4*np.log(2*np.pi)+2*np.log(np.diagonal(root,axis1=-2,axis2=-1)).sum(1)
                     +np.einsum('ni,ni->n',mean_actual,actual_h))
    increment_j = matrix(encoded[:,:,:10]); increment_h = encoded[:,:,10:14]; increment_c = encoded[:,:,14]
    old_kept = np.zeros_like(kept) if scalar else rec[:,52:54]
    base_j = actual_j-np.einsum('ns,nsij->nij',old_kept,increment_j)
    base_h = actual_h-np.einsum('ns,nsi->ni',old_kept,increment_h)
    base_c = constant+(rec[:,10] if scalar else rec[:,56])-np.sum(old_kept*increment_c,axis=1)
    j = base_j+np.einsum('ns,nsij->nij',kept,increment_j)
    h = base_h+np.einsum('ns,nsi->ni',kept,increment_h)
    c = base_c+np.sum(kept*increment_c,axis=1); j = (j+j.swapaxes(-1,-2))/2
    fallback = np.zeros(len(rec),bool)
    for i in np.flatnonzero(called):
        try:
            np.linalg.cholesky(j[i])
            fallback[i] = (not np.isfinite(j[i]).all() or not np.isfinite(h[i]).all()
                           or not np.isfinite(c[i]) or 1/np.linalg.cond(j[i],1)<1e-12)
        except np.linalg.LinAlgError: fallback[i] = True
    kept[fallback] = 0; j[fallback],h[fallback],c[fallback] = base_j[fallback],base_h[fallback],base_c[fallback]
    # Determine sign-loss events from independently rebuilt original admission.
    positive_rejected=np.zeros(len(rec),bool); negative_rejected=np.zeros(len(rec),bool)
    for i in range(len(rec)):
        rejected_signs=[np.sign(rec[i,19+s]) for s in range(2) if raw[i,s]>0 and not allowed[i,s]]
        accepted_signs=[np.sign(rec[i,19+s]) for s in range(2) if kept[i,s]>0]
        positive_rejected[i]=1 in rejected_signs and -1 in accepted_signs
        negative_rejected[i]=-1 in rejected_signs and 1 in accepted_signs
    trigger=np.zeros(len(rec),bool)
    if rule in ['paired_veto','positive_rejected_veto']: trigger |= positive_rejected
    if rule in ['paired_veto','negative_rejected_veto']: trigger |= negative_rejected
    kept[trigger]=0
    j[trigger],h[trigger],c[trigger]=base_j[trigger],base_h[trigger],base_c[trigger]
    root = np.linalg.cholesky(j)
    forward = np.linalg.solve(root,h[...,None])
    mean = np.linalg.solve(root.swapaxes(-1,-2),forward)[...,0]
    covariance = np.linalg.solve(j,np.broadcast_to(np.eye(4),j.shape))
    log_i = c+.5*(4*np.log(2*np.pi)-2*np.log(np.diagonal(root,axis1=-2,axis2=-1)).sum(1)+np.einsum('ni,ni->n',h,mean))
    log_i[~(kept>0).any(1)] = rec[~(kept>0).any(1),10]
    if scalar: mean,covariance,log_i = mean_actual,covariance_actual,rec[:,10].copy()
    logits = np.zeros_like(kept); p = np.clip(rec[:,17:19][active],1e-9,1-1e-9)
    logits[active] = np.log(p)-np.log1p(-p)
    r = expit(np.sum(rec[:,28:30]*logits,axis=1)+np.sum(kept*rec[:,19:21],axis=1)+log_i)
    # Original reconstruction must match before enforcing exact no-op storage.
    unchanged=~trigger
    assert np.allclose(r[unchanged],rec[unchanged,9],atol=2e-10,rtol=0)
    assert np.allclose(mean[unchanged],mean_actual[unchanged],atol=1e-7,rtol=0)
    assert np.allclose(covariance[unchanged],covariance_actual[unchanged],atol=1e-7,rtol=0)
    assert np.allclose(kept[unchanged],rec[unchanged,52:54],atol=2e-14,rtol=0)
    r[unchanged]=rec[unchanged,9]; mean[unchanged]=mean_actual[unchanged]
    covariance[unchanged]=covariance_actual[unchanged]
    log_i[unchanged]=rec[unchanged,10 if scalar else 56]
    return rec,dict(r=r,mean=mean,covariance=covariance,log_integral=log_i,
                    kept=kept,allowed=allowed,fallback=fallback,positive=support,
                    positive_rejected=positive_rejected,negative_rejected=negative_rejected,trigger=trigger)


def extract(rec, values, mask, poses, original):
    r = rec[mask,9] if original else values['r'][mask]
    means = rec[mask,4:6] if original else values['mean'][mask,:2]
    retained = r>.001; p = r[retained]-1e-6; means = means[retained]
    pmf = np.array([1.])
    for probability in p:
        new = np.zeros(len(pmf)+1)
        new[:-1] += pmf*(1-probability); new[1:] += pmf*probability; pmf = new
    count = int(pmf.argmax()); selected = means[np.argsort(-p,kind='stable')[:count]]
    d2 = ((selected[:,None,:]-poses.T[None,:,:])**2).sum(-1).min(1)
    inside = (abs(selected[:,0])<=70.4)&(abs(selected[:,1])<=40)&(d2<=1600)&(d2>9)
    selected = selected[inside]
    return np.c_[selected,np.zeros((len(selected),2))]


def scores(truth, estimate):
    x = np.asarray(truth,float).reshape(4,-1).T[:,:2]
    y = np.asarray(estimate,float).reshape(-1,4)[:,:2]
    distance = ((x[:,None,:]-y[None,:,:])**2).sum(-1)
    rows,columns = linear_sum_assignment(np.minimum(distance,144.))
    values = distance[rows,columns]; within = values<144.
    loc = math.fsum(values[within]); hits = int(within.sum())
    miss = 72.*(len(x)-hits); false = 72.*(len(y)-hits)
    numerator = math.fsum(np.minimum(values,144.))+144.*abs(len(x)-len(y))
    return dict(ospa=math.sqrt(numerator/max(len(x),len(y))) if max(len(x),len(y)) else 0.,
                gospa=math.sqrt(loc+miss+false),loc2=loc,miss2=miss,false2=false,
                countError=abs(len(x)-len(y)),outputCount=len(y))


def main():
    destination = OUT/'SCREEN_VERIFICATION.json'; assert not destination.exists()
    freeze = OUT/'SCREEN_FREEZE.json'; report_path = OUT/'SCREEN_RESULTS.json'
    cfg = json.loads(freeze.read_text()); report = json.loads(report_path.read_text())
    execution = json.loads((OUT/'SCREEN_EXECUTION.json').read_text())
    assert report['passed'] and execution['completed'] and execution['returncode']==0
    assert report['freeze_sha256']==execution['freeze_sha256']==sha(freeze)
    assert sha(ROOT/execution['log'])==execution['log_sha256']
    for name,digest in cfg['source_sha256'].items(): assert sha(ROOT/name)==digest,name
    for name,digest in report['artifacts'].items(): assert sha(ROOT/name)==digest,name
    original_rows = {(r['sequence'],r['condition'],r['backend'],r['rule']):r for r in report['rows']}
    rebuilt_rows = []; fusion_labels = 0; robot_frames = 0; input_shas = {}
    for index,cell in enumerate(cfg['cells']):
        path = ROOT/cell['path']; assert sha(path)==cell['sha256']
        with gzip.open(path,'rt') as stream: data=json.load(stream)
        prefix=OUT/'results'/f"{index:02d}_{cell['sequence']}_{cell['condition']}_{cell['backend'].replace(' ','_')}"
        arrays_path=prefix.with_suffix('.npz'); outputs_path=prefix.with_suffix('.json.gz')
        arrays=np.load(arrays_path)
        with gzip.open(outputs_path,'rt') as stream: outputs=json.load(stream)
        assert outputs['cell']==cell
        poses=np.asarray(data['positions']); delivery=np.asarray(data['delivered'],bool); T=len(data['time'])
        for rule in RULES:
            rec,values=rebuilt(data['runs'],rule,cell['backend']=='Guarded Scalar')
            for key,value in values.items():
                stored=arrays[rule+'__'+key]
                if key in ['allowed','fallback','positive_rejected','negative_rejected','trigger']: assert np.array_equal(value,stored),(cell,rule,key)
                else:
                    tolerance=1e-7 if key in ['mean','covariance'] else 1e-8 if key=='log_integral' else 2e-10 if key=='r' else 2e-14
                    assert np.allclose(value,stored,atol=tolerance,rtol=0),(cell,rule,key,float(np.max(abs(value-stored))))
            source_output=outputs['outputs'][rule]; sums={k:[] for k in METRICS}
            for t in range(T):
                for n in range(2):
                    mask=(rec[:,0]==t+1)&(rec[:,1]==n+1)
                    estimate=(extract(rec,values,mask,poses[:,:,t],rule=='original') if delivery[n,1-n,t]
                              else data['runs']['estimates'][n+2*t])
                    saved=np.asarray(source_output['estimates'][n+2*t],float).reshape(-1,4)
                    estimate=np.asarray(estimate,float).reshape(-1,4)
                    assert len(saved)==len(estimate)
                    assert np.allclose(estimate[:,:2],saved[:,:2],atol=1e-7,rtol=0),(cell,rule,t,n,'extracted set')
                    value=scores(data['truth'][t],estimate)
                    for key in METRICS:
                        assert abs(value[key]-source_output['scores'][key][n+2*t])<1e-7,(cell,rule,t,n,key)
                        sums[key].append(value[key])
                    robot_frames+=1
            row={k:cell[k] for k in ['dataset','sequence','condition','backend','recording']}
            row.update(rule=rule,frames=T,**{k:math.fsum(v)/len(v) for k,v in sums.items()})
            reference=original_rows[cell['sequence'],cell['condition'],cell['backend'],rule]
            for key in METRICS: assert abs(row[key]-reference[key])<1e-7
            rebuilt_rows.append(row); fusion_labels+=len(rec)
        arrays.close(); input_shas[str(path.relative_to(ROOT))]=sha(path)
        print('INDEPENDENT SUBSTITUTION VERIFIED',index+1,'/',len(cfg['cells']),cell['sequence'],cell['condition'],cell['backend'],flush=True)
    gates=[]
    for dataset in ['v2v_development','v2x_val']:
        for condition in ['reliable','intermittent']:
            values={}
            for rule in ['paired_veto','original']:
                group=[r for r in rebuilt_rows if (r['dataset'],r['backend'],r['condition'],r['rule'])==(dataset,'GCE',condition,rule)]
                assert len(group)==(9 if dataset=='v2v_development' else 5)
                values[rule]=math.fsum(r['ospa'] for r in group)/len(group)
            gate=dict(dataset=dataset,condition=condition,reference='original',difference=values['paired_veto']-values['original'],passed=values['paired_veto']<values['original'])
            expected=next(g for g in report['gates'] if (g['dataset'],g['condition'])==(dataset,condition))
            assert gate['passed']==expected['passed'] and abs(gate['difference']-expected['difference'])<1e-7
            gates.append(gate)
    with (OUT/'ALL_SCREEN_SCORES.csv').open(newline='') as stream: table=list(csv.DictReader(stream))
    assert len(table)==len(rebuilt_rows)==224
    for csvrow,row in zip(table,report['rows']):
        for key,value in row.items():
            if isinstance(value,(int,float)): assert float(csvrow[key])==value
            else: assert csvrow[key]==value
    assert report['advance_to_recursion']==all(g['passed'] for g in gates)
    assert report['robot_frames']==robot_frames and report['source_runs']==len(cfg['cells'])
    for name,digest in cfg['source_sha256'].items(): assert sha(ROOT/name)==digest,name
    result=dict(passed=True,source_runs=len(cfg['cells']),fusion_distributions=fusion_labels,
        alternate_robot_frames=robot_frames,rows=len(rebuilt_rows),gates=gates,
        advance_to_recursion=all(g['passed'] for g in gates),
        method='Saved fused density minus original transmitted residual plus new residual; Cholesky integration, separate cardinality recurrence and scores',
        report_sha256=sha(report_path),freeze_sha256=sha(freeze),verifier_sha256=sha(Path(__file__)),
        execution_sha256=sha(OUT/'SCREEN_EXECUTION.json'),csv_sha256=sha(OUT/'ALL_SCREEN_SCORES.csv'),inputs=input_shas)
    destination.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('SCREEN INDEPENDENT VERIFICATION PASSED',fusion_labels,'distributions',robot_frames,'robot frames',flush=True)


if __name__=='__main__':main()
