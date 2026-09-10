"""Create new orchestration/verifier files through exact, recorded source edits."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
OLD=OUT.parent/'icra_joint_admission'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
edits=[]


def replace(text,old,new):
    assert text.count(old)==1,old
    edits.append(dict(old=old,new=new))
    return text.replace(old,new)


def write(name,text,source):
    destination=OUT/name; assert not destination.exists()
    destination.write_text(text)
    return dict(source=str(source.relative_to(OUT.parents[1])),source_sha256=sha(source),
                output_sha256=sha(destination),replacements=edits.copy())


def main():
    ports={}
    source=OLD/'screen.py'; text=source.read_text(); edits.clear()
    text=replace(text,'from scipy.io import loadmat\n','')
    text=replace(text,'from joint_math import RULES, check_fixtures, source_state, calculate, audit_raw_weights',
                 'from conflict_math import RULES, check_fixtures, source_state, calculate')
    text=replace(text,"cfg['primary'] == 'joint'","cfg['primary'] == 'paired_veto'")
    text=replace(text,"        ratio = loadmat(ROOT / cell['ratios_path'])['likelihoodRatios']\n        raw = audit_raw_weights(data, state, ratio)",
                 "        raw = dict(rechecked_here=False, earlier_audit='icra_joint_admission/WEIGHT_ARITHMETIC_VERIFICATION.json')")
    start=text.index('    gates = []\n'); end=text.index('    for name, digest',start)
    text=replace(text,text[start:end],'''    gates = []
    for dataset in ['v2v_development','v2x_val']:
        for condition in cfg['conditions']:
            primary=lookup[dataset,'GCE','paired_veto']['conditions'][condition]['ospa']
            ref=lookup[dataset,'GCE','original']['conditions'][condition]['ospa']
            gates.append(dict(dataset=dataset,condition=condition,reference='original',
                candidate_mean=primary,reference_mean=ref,difference=primary-ref,passed=primary<ref))
''')
    ports['screen.py']=write('screen.py',text,source)

    source=OLD/'execute_screen.py'; text=source.read_text(); edits.clear()
    text=replace(text,'RUN/ICRA_JOINT_ADMISSION/screen.log','RUN/ICRA_CURVATURE_CONFLICT/screen.log')
    ports['execute_screen.py']=write('execute_screen.py',text,source)

    source=OLD/'verify_screen.py'; text=source.read_text(); edits.clear()
    text=replace(text,"RULES = ['original','joint_mark','conditional','joint']",
                 "RULES = ['original','paired_veto','positive_rejected_veto','negative_rejected_veto']")
    text=replace(text,"            support[i,s] = (row[8] if rule == 'original' else row[5]*row[8] if rule == 'joint_mark'\n                            else row[9] if rule == 'conditional' else row[5]*row[9])", "            support[i,s] = row[8]")
    insertion='''    # Determine sign-loss events from independently rebuilt original admission.
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
'''
    text=replace(text,'    root = np.linalg.cholesky(j)\n',insertion+'    root = np.linalg.cholesky(j)\n')
    text=replace(text,"    return rec,dict(r=r,mean=mean,covariance=covariance,log_integral=log_i,\n                    kept=kept,allowed=allowed,fallback=fallback,positive=support)",
'''    # Original reconstruction must match before enforcing exact no-op storage.
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
                    positive_rejected=positive_rejected,negative_rejected=negative_rejected,trigger=trigger)''')
    text=replace(text,"if key in ['allowed','fallback']:","if key in ['allowed','fallback','positive_rejected','negative_rejected','trigger']:")
    start=text.index('    gates=[]\n'); end=text.index("    with (OUT/'ALL_SCREEN_SCORES.csv')",start)
    text=replace(text,text[start:end],'''    gates=[]
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
''')
    ports['verify_screen.py']=write('verify_screen.py',text,source)
    destination=OUT/'SCREEN_PORT.json'; assert not destination.exists()
    destination.write_text(json.dumps(ports,indent=2)+'\n')
    print('EXACT SOURCE PORTS',len(ports),flush=True)


if __name__=='__main__':main()
