"""Audit the common runner on previously seen sequence 0000 only."""
from pathlib import Path
import gzip,hashlib,json,re,sys
import numpy as np
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
sys.path.insert(0,str(OUT.parent/'icra_external_fusion'))
from analyze_case_studies import score
from analyze_v2v4real import domain
ARMS=['local','lineage','qualified_exist','mil_support','tc_ospa2_w5','tc_ospa2_w10','ceiling_calibrated',
      'marked_local','marked_lineage','marked_er','marked_mil_support','marked_tc_ospa2_w5','marked_tc_ospa2_w10',
      'marked_ceiling_association','marked_ceiling_score','marked_ceiling_calibrated']


def read(path):
    with gzip.open(path,'rt') as f:return json.load(f)


def main():
    source=json.loads((OUT/'source_sha256_port.json').read_text())
    for name,h in source.items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==h,name
    matlab={re.sub('[^A-Za-z0-9_]','_',p)[:63]:h for p,h in source.items()};assert len(matlab)==len(source)
    rows=[];inputs=[];audited=0;parities=[];local_cache={}
    for condition in ['reliable','intermittent']:
        old=read(OUT.parent/'icra_external_fusion/results_v2v'/f'0000_{condition}.json.gz')
        reference={r['arm']:r for r in old['runs']}
        ecr=read(OUT.parent/'icra_ceiling_iteration/results_development'/f'0000_{condition}.json.gz')
        reference.update({r['arm']:r for r in ecr['runs'] if r['arm']=='ceiling_calibrated'})
        marked=read(OUT.parent/'icra_marked_iteration/results_stable'/f'0000_{condition}.json.gz')
        reference.update({r['arm']:r for r in marked['runs']})
        for arm in ARMS:
            path=OUT/'results_development_check'/f'0000_{condition}_{arm}.json.gz';data=read(path);run=data['runs']
            assert data['implementation']=='common-port-v1' and data['cohort']=='development_check' and data['sourceSha256']==matlab
            assert run['arm']==arm and not data['smoke']
            for key in ['truth','truthIds','positions','time','delivered','inputSha256']:assert data[key]==old[key],key
            T=len(data['time']);poses=np.asarray(data['positions']);delivery=np.asarray(data['delivered'])
            packets=np.asarray(run['packetBytes']);assert packets.shape==(2,T)
            if arm.endswith('local'):
                assert not packets.any() and not any(run['wireBytes'])
                if arm in local_cache:assert run['rawEstimates']==local_cache[arm]
                else:local_cache[arm]=run['rawEstimates']
            else:
                assert run['attemptedMessages']==[2]*T and run['controlBytes']==[256]*T
                assert run['deliveredMessages']==delivery.sum((0,1)).tolist()
                assert run['rawPayloadBytes']==packets.sum(0).tolist()
                assert run['deliveredRawBytes']==(delivery.sum(0)*packets).sum(0).tolist()
                assert run['wireBytes']==(np.ceil(packets/16384).sum(0)*16384+256).tolist()
                if 'tc_ospa2' not in arm:
                    width=216 if 'ceiling_' in arm else 208
                    assert ((packets-32)%width==0).all()
            assert run['totalWireBytes']==sum(run['wireBytes'])
            if arm in reference:
                previous=reference[arm];exact=True;maximum_difference=0.
                for key in ['estimates','rawEstimates','labels','ospa','countError','matchedSquaredError','matchedCount']:
                    a,b=run[key],previous[key];exact=exact and a==b
                    assert len(a)==len(b)
                    for aa,bb in zip(a,b):
                        aa=np.asarray(aa);bb=np.asarray(bb);assert aa.shape==bb.shape,(arm,key)
                        assert np.allclose(aa,bb,rtol=1e-10,atol=1e-9),(arm,key,'port parity')
                        if aa.size:maximum_difference=max(maximum_difference,float(np.max(np.abs(aa-bb))))
                if not arm.startswith('marked_'):
                    for key in ['rawPayloadBytes','deliveredRawBytes','wireBytes']:assert run[key]==previous[key],(arm,key)
                elif 'ceiling_' in arm:
                    for key in ['rawPayloadBytes','deliveredRawBytes','wireBytes','iterationRecords']:assert run[key]==previous[key],(arm,key)
                else:
                    count=(np.asarray(previous['rawPayloadBytes'])-64)/216
                    assert run['rawPayloadBytes']==(64+count*208).tolist()
                parities.append(dict(condition=condition,arm=arm,exact_output_values=exact,maximum_numeric_difference=maximum_difference,node_frames=2*T))
            metrics={key:[] for key in ['ospa','gospa','loc2','miss2','false2','countError']}
            for t in range(T):
                for n in range(2):
                    index=n+2*t;raw=run['rawEstimates'][index];output=run['estimates'][index]
                    assert np.array_equal(np.asarray(output).reshape(-1,4),np.asarray(raw).reshape(-1,4)[domain(raw,poses[:,:,t])])
                    value=score(data['truth'][t],output);audited+=1
                    for key in ['ospa','countError','matchedSquaredError','matchedCount']:
                        assert np.isclose(value[key],run[key][n][t],atol=1e-8,rtol=1e-9),(arm,key)
                    for key in metrics:metrics[key].append(value[key])
            rows.append(dict(condition=condition,arm=arm,**{k:float(np.mean(v)) for k,v in metrics.items()},
                             raw_bytes=sum(run['rawPayloadBytes']),wire_bytes=sum(run['wireBytes'])))
            inputs.append(dict(file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
            print('PORT AUDITED',condition,arm,audited,flush=True)
    assert len(rows)==32 and audited==9408 and len(parities)==24
    report=dict(scope='Seen sequence 0000 only; no holdout tracking outcomes.',source_hashes_verified=len(source),
                audited_node_frames=audited,paired_prior_node_frames=sum(r['node_frames'] for r in parities),
                parity=parities,per_source_packet_accounting_verified=True,runs=rows,inputs=inputs)
    (OUT/'port_audit.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('PORT AUDIT PASSED',audited,'nodes;',sum(r['exact_output_values'] for r in parities),'of',len(parities),'comparisons bitwise equal.')


if __name__=='__main__':main()
