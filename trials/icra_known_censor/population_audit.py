"""Check native birth/prediction/pruning balances and enumerate reimport cycles."""
from collections import defaultdict
import numpy as np
from trace_v2 import prepare,rows_at,pool_at
from review_gaussian_audit import unpack

A=np.block([[np.eye(2),.1*np.eye(2)],[np.zeros((2,2)),np.eye(2)]])
Q=25*np.block([[(.1**3/3)*np.eye(2),(.1**2/2)*np.eye(2)],[(.1**2/2)*np.eye(2),.1*np.eye(2)]])

def keyed(rows):
    out={tuple(map(int,r[2:4])):r for r in rows};assert len(out)==len(rows);return out

def audit_population(data,mat):
    p=prepare(data);previous={1:{},2:{}};cycles={1:{},2:{}};events=[];population=[];predictions=0;covariances=0
    for t in range(1,241):
        local={};inc={};diagnostics={}
        for n in [1,2]:
            inc[n]=keyed(rows_at(p,'inc',t,n));pred=keyed(rows_at(p,'predicted',t,n));gauss=keyed(rows_at(p,'gaussian',t,n))
            local[n]=keyed(rows_at(p,'local',t,n));z=np.asarray(mat['measurements'][n-1,t-2]).reshape(2,-1) if t>1 else np.zeros((2,0))
            births={(t,n*100000+j+1):np.r_[z[:,j],0.,0.] for j in range(z.shape[1])}
            assert not previous[n].keys()&births.keys()
            assert inc[n].keys()==pred.keys()==previous[n].keys()|births.keys()
            assert gauss.keys()==local[n].keys()=={k for k,r in inc[n].items() if r[5]>.001}
            for k,r in inc[n].items():
                if k in births:er=.01;mean=births[k];cov=np.diag([16.,16.,225.,225.])
                else:
                    old=previous[n][k];er=.99*old[4];mean=A@old[5:9];cov=A@unpack(old[9:])@A.T+Q
                assert abs(r[4]-er)<1e-12 and np.allclose(pred[k][5:9],mean,atol=1e-7,rtol=0)
                predictions+=1
                if k in gauss:
                    assert np.allclose(gauss[k][4:8],mean,atol=1e-7,rtol=0)
                    assert np.allclose(unpack(gauss[k][8:18]),cov,atol=1e-7,rtol=0);covariances+=1
            diagnostics[n]=dict(frame=t,robot=n,previous=len(previous[n]),births=len(births),predicted=len(inc[n]),
                local=len(local[n]),local_pruned=len(inc[n])-len(local[n]))
        current={}
        ids=np.asarray(data['truthIds'][t-1]).ravel();xy=np.asarray(data['truth'][t-1],float).reshape(4,-1)[:2,np.flatnonzero(ids==5).item()]
        for n in [1,2]:
            receive=bool(data['delivered'][n-1][2-n][t-1]);fused=keyed(pool_at(p,t,n));sources=keyed(rows_at(p,'source_labels',t,n));records=keyed(rows_at(p,'records',t,n))
            used=[set(),set()];newcycles={}
            if receive:
                assert fused.keys()==sources.keys()==records.keys()
                for k,s in sources.items():
                    parents=s[4:].astype(int).reshape(2,2);assert (parents[:,0]>0).any()
                    for side in [0,1]:
                        if parents[side,0]==0:assert not parents[side].any();continue
                        key=tuple(parents[side]);sensor=n if side==0 else 3-n
                        assert key in local[sensor] and key not in used[side];used[side].add(key)
                        if side==0:assert key==k
                        if records[k][13+side]>0:assert abs(records[k][17+side]-inc[sensor][key][5])<1e-9
                assert used[0]==local[n].keys() and used[1]==local[3-n].keys()
            else:assert not sources and not records and fused.keys()==local[n].keys()
            current[n]={k:r for k,r in fused.items() if r[4]>.001}
            for k,r in current[n].items():
                if k in local[n]:continue
                assert receive and sources[k][4]==0 and sources[k][6]>0
                known=inc[n].get(k);same=known is not None
                if same:
                    assert known[5]<=.001;newcycles[k]=cycles[n].get(k,0)+1
                record=records[k];qualified=same and record[13]>0
                events.append(dict(frame=t,robot=n,birth_frame=k[0],birth_location=k[1],same_frame_deleted=same,
                    qualified_censor=qualified,current_opportunity=bool(known[7]) if same else False,
                    local_r=float(known[5]) if same else None,returned_r=float(r[4]),
                    near_target=bool(np.sum((r[5:7]-xy)**2)<=4),consecutive_returns=newcycles.get(k,0)))
            cycles[n]=newcycles
            d=diagnostics[n];d.update(received=receive,pre_fusion_prune=len(fused),fusion_pruned=len(fused)-len(current[n]),
                retained=len(current[n]),local_lost_at_fusion=len(local[n].keys()-current[n].keys()),
                remote_only_retained=len(current[n].keys()-local[n].keys()),delta=len(current[n])-len(previous[n]))
            assert d['delta']==d['births']-d['local_pruned']-d['local_lost_at_fusion']+d['remote_only_retained'];population.append(d)
        previous=current
    summaries=[]
    for window,left,right in [('full',1,240),('original_window',53,122)]:
        for scope in ['all','target_neighbourhood']:
            selected=[e for e in events if left<=e['frame']<=right and (scope=='all' or e['near_target'])]
            deleted=[e for e in selected if e['same_frame_deleted']]
            summaries.append(dict(window=window,scope=scope,remote_only_retained=len(selected),same_frame_deleted=len(deleted),
                qualified_censor=sum(e['qualified_censor'] for e in deleted),
                qualified_with_opportunity=sum(e['qualified_censor'] and e['current_opportunity'] for e in deleted),
                at_least_two_consecutive_returns=sum(e['consecutive_returns']>=2 for e in deleted),
                maximum_consecutive_returns=max((e['consecutive_returns'] for e in deleted),default=0)))
    assert len(population)==480
    return dict(predictions_checked=predictions,available_prediction_covariances_checked=covariances,
        population_rows=population,reimport_summaries=summaries,reimport_events=events)
