"""Keep the complete previous audit, adapting explicit new bytes and operands."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    source=OUT.parent/'icra_known_censor/audit_stage_v2.py';target=OUT/'audit_stage.py';assert not target.exists()
    before=source.read_text();start=before.index('def exact_prefix(');end=before.index('\ndef main():')
    prefix='''def exact_prefix(run,control):
    events=np.asarray(run['pruneInformationRecords'],float).reshape(-1,24)
    changed=events[(events[:,6]==2)&(events[:,4]!=events[:,5])]
    assert len(changed);first=int(changed[:,0].min());checked=[]
    for name in ['localIncrementRecords','localGaussianRecords','qualityRecords','packetGaussianRecords',
                 'localDirectRecords','packetDirectRecords','fusionSourceRecords']:
        assert [r for r in run[name] if r[0]<=first]==[r for r in control[name] if r[0]<=first],name;checked.append(name)
    for name,omit in [('iterationRecords',{6,9}),('fusionOutputRecords',{4})]:
        assert [r for r in run[name] if r[0]<first]==[r for r in control[name] if r[0]<first],name
        a=[[x for i,x in enumerate(r) if i not in omit] for r in run[name] if r[0]==first]
        b=[[x for i,x in enumerate(r) if i not in omit] for r in control[name] if r[0]==first]
        assert a==b,name;checked.append(name+'_except_actual_r_at_first_peer_event')
    for name in ['estimates','rawEstimates','labels']:
        assert run[name][:2*(first-1)]==control[name][:2*(first-1)],name;checked.append(name)
    assert run['localAssociationWeights'][:2*first]==control['localAssociationWeights'][:2*first];checked.append('localAssociationWeights')
    for name in ['basePacketBytes','matchedLabels']:
        assert [r[:first] for r in run[name]]==[r[:first] for r in control[name]],name;checked.append(name)
    for name in ['controlBytes','attemptedMessages','deliveredMessages']:
        assert run[name]==control[name],name;checked.append(name)
    return dict(first_changed_frame=first,exact_prefix_fields=checked)

'''
    changes=[
        ("'icra_temporal_association','icra_miss_history','icra_range_detection','icra_recursion_origin'", "'icra_temporal_association','icra_miss_history','icra_range_detection','icra_recursion_origin','icra_known_censor'"),
        ('from audit_screen_assessment_v2 import packets','from prune_event_audit import packets,eligible_rows\nfrom event_audit import eligible_rows as local_eligible_rows'),
        ('from event_audit import eligible_rows\n',''),
        ("SUFFIX='_known_censor'","SUFFIX='_prune_info'"),
        (before[start:end],prefix),
        ("choices=['known_censor_controls','known_censor_refined']","choices=['prune_info_controls','prune_info_shared']"),
        ("destination=OUT/('audit_v2_'+args.stage+'.json')","destination=OUT/('audit_'+args.stage+'.json')"),
        ("    v2=json.loads((OUT/'AUDIT_FREEZE_V2.json').read_text())\n    for name,h in v2['source_sha256'].items():assert sha(ROOT/name)==h,name\n",''),
        ("'RUN/ICRA_KNOWN_CENSOR'","'RUN/ICRA_PRUNE_INFORMATION'"),
        ("['KNOWN CENSOR CHECK PASSED','RANGE DETECTION CHECK PASSED',", "['PRUNE INFORMATION CHECK PASSED','KNOWN CENSOR CHECK PASSED','RANGE DETECTION CHECK PASSED',"),
        ('base=arm.removesuffix(SUFFIX)',"base=arm.removesuffix(SUFFIX).removesuffix('_known_censor')"),
        ("run['knownCensorEnabled']==arm.endswith(SUFFIX)","run['knownCensorEnabled']==arm.endswith('_known_censor') and run['pruneInformationEnabled']==arm.endswith(SUFFIX)"),
        ("quality=audit_actual_pd(run,data);view=source_view(packets(run,delivered,240,''));view['arm']=base",
         "quality=audit_actual_pd(run,data);packet_view,communication=packets(run,delivered,240);view=source_view(packet_view);view['arm']=base"),
        ("pop=audit_population(data,mat);ix,_=eligible_rows(view,np.asarray(view['iterationRecords']).reshape(-1,60))\n            events=np.asarray(run['knownCensorRecords'],float).reshape(-1,25)\n            assert len(events)==(len(ix) if run['knownCensorEnabled'] else 0)",
         "pop=audit_population(data,mat)\n            if run['pruneInformationEnabled']:\n                ix,_=eligible_rows(view,np.asarray(view['iterationRecords']).reshape(-1,60))\n                events=np.asarray(run['pruneInformationRecords'],float).reshape(-1,24)\n            else:\n                ix,_=local_eligible_rows(view,np.asarray(view['iterationRecords']).reshape(-1,60))\n                events=np.asarray(run['knownCensorRecords'],float).reshape(-1,25)\n            assert len(events)==len(ix)"),
        ("_reimport_v2.csv.gz","_reimport.csv.gz"),
        ('quality=quality,density=density,direct=direct,positive_mark_rows=marks,','quality=quality,density=density,communication=communication,direct=direct,positive_mark_rows=marks,'),
        ("if args.stage=='known_censor_controls':","if args.stage=='prune_info_controls':"),
        ("ref=u['references'][base]","ref=u['references'][arm]"),
        ("'results/known_censor_controls'/f'{seq}_{condition}_{base}.json.gz'","'results/prune_info_controls'/f'{seq}_{condition}_{base}_known_censor.json.gz'"),
        ("'audit_v2_known_censor_controls.json'","'audit_prune_info_controls.json'"),
        ('AUDITED KNOWN CENSOR','AUDITED PRUNE INFORMATION'),
        ("[('scores_v2_',rows),('target_frames_v2_',timeline),('population_frames_v2_',population_rows)]","[('scores_',rows),('target_frames_',timeline),('population_frames_',population_rows)]"),
        ('KNOWN CENSOR NATIVE AUDIT PASSED','PRUNE INFORMATION NATIVE AUDIT PASSED'),
    ]
    content=before
    for a,b in changes:assert content.count(a)==1,a;content=content.replace(a,b)
    target.write_text(content)
    (OUT/'AUDIT_PORT.json').write_text(json.dumps(dict(source=str(source.relative_to(ROOT)),source_sha256=sha(source),
        target=str(target.relative_to(ROOT)),target_sha256=sha(target),changes=[dict(before=a,after=b) for a,b in changes]),indent=2)+'\n')
    print('PRUNE INFORMATION AUDIT GENERATED',len(changes),flush=True)

if __name__=='__main__':main()
