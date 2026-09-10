"""Independently decode packet bytes and reconstruct all final scalar operands."""
from collections import defaultdict
import struct
import numpy as np
from scipy.special import expit
from event_audit import check_censor_event as check_receiver_only

def packets(run,delivered,T):
    local=np.asarray(run['localGaussianRecords'],float).reshape(-1,32);counts=np.zeros((2,T),int)
    for row in local:counts[int(row[1])-1,int(row[0])-1]+=1
    base=32+(352 if 'gaussian_evidence' in run['arm'] else 216)*counts
    assert np.array_equal(base,run['basePacketBytes'])
    inc=np.asarray(run['localIncrementRecords'],float).reshape(-1,12)
    lookup={tuple(map(int,r[:4])):r for r in inc};assert len(lookup)==len(inc)
    filtered=defaultdict(list)
    for r in inc:
        if r[5]<=.001 and r[7]:
            assert r[10]>0;filtered[int(r[0]),int(r[1])].append([r[2],r[3],r[5],r[10]])
    rows={};sizes=np.zeros((2,T),int);raw=run['pruneTrailerBytes'];assert len(raw)==2*T
    enabled=run['pruneInformationEnabled'];assert isinstance(enabled,bool)
    for t in range(1,T+1):
        for n in [1,2]:
            logged=raw[n-1+2*(t-1)]
            if not enabled:assert logged==[];continue
            assert all(isinstance(v,int) and 0<=v<=255 for v in logged)
            data=bytes(logged);assert len(data)>=32 and len(data)%32==0
            magic,sender,frame,count=struct.unpack('<4d',data[:32]);assert (magic,sender,frame)==(73190501,n,t)
            assert count>=0 and count==int(count) and len(data)==32+32*count
            decoded=list(struct.iter_unpack('<4d',data[32:]));expected=filtered[t,n]
            assert decoded==[tuple(r) for r in expected],'all and only current pruned sender rows'
            values=[73190501,n,t,len(expected),*[v for r in expected for v in r]]
            assert data==struct.pack('<'+'d'*len(values),*values),'independent byte reconstruction'
            for bt,bl,r,pd in decoded:
                assert 1<=bt<=t and bt==int(bt) and bl>=1 and bl==int(bl) and 0<=r<=.001 and 0<pd<=1
                key=(t,n,int(bt),int(bl));assert key not in rows;rows[key]=(r,pd)
            sizes[n-1,t-1]=len(data)
    expected=base+sizes
    assert np.array_equal(expected,run['packetBytes'])
    assert run['attemptedMessages']==[2]*T and run['controlBytes']==[256]*T
    assert np.array_equal(run['deliveredMessages'],delivered.sum((0,1)))
    assert np.array_equal(run['rawPayloadBytes'],expected.sum(0))
    assert np.array_equal(run['deliveredRawBytes'],(expected*delivered.sum(0)).sum(0))
    assert np.array_equal(run['wireBytes'],np.ceil(expected/16384).sum(0)*16384+256)
    assert run['totalWireBytes']==sum(run['wireBytes'])
    summary=dict(attempted_trailer_bytes=int(sizes.sum()),delivered_trailer_bytes=int((sizes*delivered.sum(0)).sum()),
        reported_pruned_rows=len(rows),checked_packets=2*T,header_bytes=32*2*T if enabled else 0,
        attempted_base_bytes=int(base.sum()),attempted_payload_bytes=int(expected.sum()))
    view=dict(run,packetBytes=base.tolist(),_prune_report_lookup=rows,_prune_local_lookup=lookup,_prune_delivered=delivered)
    return view,summary

def eligible_rows(run,records):
    sources=np.asarray(run['fusionSourceRecords'],float).reshape(-1,8)
    assert np.array_equal(sources[:,:4],records[:,:4]);selected=[];operands=[]
    for i,(r,s) in enumerate(zip(records,sources)):
        present=s[4:8].reshape(2,2)[:,0]>0
        if present.all():continue
        assert present.any();side=int(np.flatnonzero(~present).item())
        if r[13+side]<=0:continue
        t,n,bt,bl=map(int,r[:4]);sensor=n if side==0 else 3-n;key=(t,sensor,bt,bl)
        if side==0:
            current=run['_prune_local_lookup'].get(key)
            if current is None or current[5]>.001 or not current[7]:continue
            values=(current[5],current[10])
        else:
            assert run['_prune_delivered'][n-1][sensor-1][t-1]
            values=run['_prune_report_lookup'].get(key)
            if values is None:continue
        assert r[17+side]==.001 and r[14-side]>0 and values[1]>0
        selected.append(i);operands.append((side,*values))
    return np.asarray(selected,int),np.asarray(operands,float).reshape(-1,3)

def check_censor_event(run,records,original_r,beta,logits,kept,delta,log_i):
    if not run['pruneInformationEnabled']:
        assert run['knownCensorEnabled'] and not run['pruneInformationRecords']
        return check_receiver_only(run,records,original_r,beta,logits,kept,delta,log_i)
    assert not run['knownCensorEnabled'] and not run['knownCensorRecords']
    events=np.asarray(run['pruneInformationRecords'],float).reshape(-1,24)
    ix,known=eligible_rows(run,records);assert len(ix)==len(events) and np.isfinite(events).all()
    assert np.array_equal(events[:,:4],records[ix,:4]) and not kept[ix].any()
    b=records[ix,13:15];q=records[ix,11:13];newlo=logits[ix].copy();side=known[:,0].astype(int)
    rr=np.clip(known[:,1],1e-9,1-1e-9);newlo[np.arange(len(ix)),side]=np.log(rr)-np.log1p(-rr)
    newage=((q-b)*newlo).sum(1);oldage=((q-b)*logits[ix]).sum(1);newbeta=b.copy()
    if 'lineage' not in run['arm']:newbeta[newage < -1e-12]=q[newage < -1e-12]
    oldinherited=(beta[ix]*logits[ix]).sum(1);newinherited=(newbeta*newlo).sum(1)
    refined=expit(newinherited+log_i[ix]);equal=known[:,1]==records[ix,17+side];refined[equal]=original_r[ix[equal]]
    assert np.all(refined<=original_r[ix]+2e-10)
    expected=original_r.copy();expected[ix]=refined
    values=np.c_[original_r[ix],refined,side+1,known[:,1:3],records[ix,17:19],b,q,beta[ix],newbeta,
                 oldage,newage,log_i[ix],oldinherited,newinherited]
    tolerance=np.array([2e-10,2e-10,0,0,0,0,0,0,0,0,0,2e-14,2e-14,2e-14,2e-14,1e-10,1e-10,1e-8,1e-10,1e-10])
    assert np.all(abs(events[:,4:]-values)<=tolerance),'executed current pruning operands'
    assert np.array_equal(events[:,5],records[ix,6]) and np.array_equal(events[:,5],records[ix,9])
    return expected
