"""Port complete orchestration and an independent encoded-ratio verifier."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
OLD=OUT.parent/'icra_joint_admission'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    receipts=[]
    def write(source,name,changes):
        target=OUT/name;assert not target.exists();text=source.read_text()
        for a,b in changes:assert text.count(a)==1,a;text=text.replace(a,b)
        target.write_text(text);receipts.append(dict(source=str(source.relative_to(ROOT)),source_sha256=sha(source),
            target=str(target.relative_to(ROOT)),target_sha256=sha(target),changes=[dict(before=a,after=b) for a,b in changes]))
    source=OLD/'screen.py';text=source.read_text();start=text.index('    gates = []\n');end=text.index('    for name, digest',start)
    gates='''    gates = []
    for dataset in ['v2v_development','v2x_val']:
        for condition in cfg['conditions']:
            for metric in ['ospa','gospa']:
                primary=lookup[dataset,'GCE','peer_joint']['conditions'][condition][metric]
                ref=lookup[dataset,'GCE','original']['conditions'][condition][metric]
                gates.append(dict(dataset=dataset,condition=condition,metric=metric,reference='original',
                    candidate_mean=primary,reference_mean=ref,difference=primary-ref,
                    passed=primary<ref if metric=='ospa' else primary<=ref))
'''
    write(source,'screen.py',[
        ('from joint_math import RULES, check_fixtures, source_state, calculate, audit_raw_weights','from peer_math import RULES, check_fixtures, source_state, calculate, audit_raw_weights'),
        ("cfg['primary'] == 'joint'","cfg['primary'] == 'peer_joint'"),(text[start:end],gates)])
    write(OLD/'execute_screen.py','execute_screen.py',[
        ('RUN/ICRA_JOINT_ADMISSION/screen.log','RUN/ICRA_PEER_DETECTION/screen.log')])
    source=OLD/'verify_screen.py';text=source.read_text();start=text.index('    gates=[]\n');end=text.index("    with (OUT/'ALL_SCREEN_SCORES.csv')",start)
    gates='''    gates=[]
    for dataset in ['v2v_development','v2x_val']:
        for condition in ['reliable','intermittent']:
            for metric in ['ospa','gospa']:
                values={}
                for rule in ['peer_joint','original']:
                    group=[r for r in rebuilt_rows if (r['dataset'],r['backend'],r['condition'],r['rule'])==(dataset,'GCE',condition,rule)]
                    assert len(group)==(9 if dataset=='v2v_development' else 5)
                    values[rule]=math.fsum(r[metric] for r in group)/len(group)
                gate=dict(dataset=dataset,condition=condition,metric=metric,reference='original',
                    difference=values['peer_joint']-values['original'],
                    passed=values['peer_joint']<values['original'] if metric=='ospa' else values['peer_joint']<=values['original'])
                expected=next(g for g in report['gates'] if (g['dataset'],g['condition'],g['metric'])==(dataset,condition,metric))
                assert gate['passed']==expected['passed'] and abs(gate['difference']-expected['difference'])<1e-7
                gates.append(gate)
'''
    oldsupport="""            support[i,s] = (row[8] if rule == 'original' else row[5]*row[8] if rule == 'joint_mark'
                            else row[9] if rule == 'conditional' else row[5]*row[9])"""
    support="""            support[i,s]=row[8];conditional[i,s]=row[9];peer_joint[i,s]=row[5]*row[9]"""
    newraw="""    conditional[~active]=0;peer_joint[~active]=0
    retention=np.ones_like(support)
    for i in range(len(rec)):
        for s in range(2):
            if rule=='no_negative':retention[i,s]=0.
            elif rule=='peer_conditional':retention[i,s]=1-conditional[i,1-s]
            elif rule=='peer_joint':retention[i,s]=1-peer_joint[i,1-s]
    negative=rec[:,35:37]*retention
    assert np.all((retention>=0)&(retention<=1))
    raw = joint[:,None]*(active-rec[:,28:30])*np.where(rec[:,19:21]>=0,support,negative)"""
    result="""    unchanged=np.all(kept==rec[:,52:54],axis=1)
    assert np.allclose(r[unchanged],rec[unchanged,9],atol=2e-10,rtol=0)
    assert np.allclose(mean[unchanged],mean_actual[unchanged],atol=1e-7,rtol=0)
    assert np.allclose(covariance[unchanged],covariance_actual[unchanged],atol=1e-7,rtol=0)
    if rule=='original':assert unchanged.all()
    r[unchanged]=rec[unchanged,9];mean[unchanged]=mean_actual[unchanged];covariance[unchanged]=covariance_actual[unchanged]
    log_i[unchanged]=rec[unchanged,10 if scalar else 56]
    return rec,dict(r=r,mean=mean,covariance=covariance,log_integral=log_i,
                    kept=kept,allowed=allowed,fallback=fallback,positive=support,negative=negative,
                    retention=retention,conditional=conditional,peer_joint=peer_joint,unchanged=unchanged)"""
    write(source,'verify_screen.py',[
        ("RULES = ['original','joint_mark','conditional','joint']","RULES = ['original','peer_conditional','peer_joint','no_negative']"),
        ('    good = np.ones(present.shape,bool)','    conditional=np.zeros_like(support);peer_joint=np.zeros_like(support)\n    good = np.ones(present.shape,bool)'),
        (oldsupport,support),
        ('    raw = joint[:,None]*(active-rec[:,28:30])*np.where(rec[:,19:21]>=0,support,rec[:,35:37])',newraw),
        ("    return rec,dict(r=r,mean=mean,covariance=covariance,log_integral=log_i,\n                    kept=kept,allowed=allowed,fallback=fallback,positive=support)",result),
        ("if key in ['allowed','fallback']:","if key in ['allowed','fallback','unchanged']:"),(text[start:end],gates)])
    (OUT/'PORTS.json').write_text(json.dumps(receipts,indent=2)+'\n');print('PEER DETECTION PORTS',len(receipts),flush=True)

if __name__=='__main__':main()
