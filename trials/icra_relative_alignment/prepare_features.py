"""Stream missing public ZIP entries into verified, compact current-frame grids."""
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path
import hashlib
import io
import json
import struct
import sys
import time
import zlib
import numpy as np
from common import OUT,ROOT,sha,write_new,frozen
import alignment_math as method
import independent_math as independent

sys.path.insert(0,str(OUT.parent/'icra_reviewer_revision'))
from fetch_official_data import Archive

def parse_cloud(raw,pcd):
    if not pcd:
        assert len(raw)%16==0
        cloud=np.frombuffer(raw,dtype='<f4').reshape(-1,4)[:,:3]
        cloud=cloud[~np.isnan(cloud).any(axis=1)]
    else:
        stream=io.BytesIO(raw);header={}
        while True:
            line=stream.readline().decode('ascii').strip();assert line
            if not line.startswith('#'):
                key,*values=line.split();header[key]=values
            if line.startswith('DATA '):break
        assert header['DATA']==['ascii'] and header['FIELDS']==['x','y','z','rgb']
        assert header['TYPE']==['F']*4 and header['SIZE']==['4']*4 and header['COUNT']==['1']*4
        values=np.loadtxt(stream,dtype=np.float32,ndmin=2)
        assert values.shape==(int(header['POINTS'][0]),4)
        cloud=values[:,:3]
    assert len(cloud)>0 and np.isfinite(cloud).all()
    return cloud.astype(float)

def extract_raw(blob,start,info):
    entry=info['entry'];offset=entry['offset']-start
    fields=struct.unpack_from('<4s5H3I2H',blob,offset)
    assert fields[0]==b'PK\x03\x04' and fields[3]==entry['compression']
    name=blob[offset+30:offset+30+fields[-2]].decode('utf-8')
    assert name==entry['name']
    begin=offset+30+fields[-2]+fields[-1]
    payload=blob[begin:begin+entry['compressed']];assert len(payload)==entry['compressed']
    assert entry['compression'] in [0,8]
    return zlib.decompress(payload,-15) if entry['compression']==8 else payload

def prepare_one(raw,info):
    digest=hashlib.sha256(raw).hexdigest()
    if 'entry' in info:
        assert len(raw)==info['entry']['raw'] and zlib.crc32(raw)==info['entry']['crc']
    else:assert len(raw)==info['bytes'] and digest==info['sha256']
    begin=time.perf_counter();points=parse_cloud(raw,'entry' in info);parse_seconds=time.perf_counter()-begin
    transform=np.array(info['transform']);begin=time.perf_counter()
    projected=method.project(points,transform);grid=method.occupancy(projected)
    encoded=method.packet(grid,info['source'],info['frame'])
    decoded=method.decode(encoded,info['source'],info['frame']);assert np.array_equal(decoded,grid)
    grid_seconds=time.perf_counter()-begin
    # The normative sums above determine which side of a voxel boundary a
    # value lies on. Check geometry separately, then index the same values.
    separate=np.einsum('nj,ij->ni',np.column_stack([points,np.ones(len(points))]),transform)[:,:3]
    error=float(np.max(np.abs(separate-projected)));assert error<=1e-9
    assert np.array_equal(independent.occupancy(projected),grid)
    row=dict(index=info['index'],sequence=info['sequence'],frame=info['frame'],source=info['source'],
             raw_path=info['path'],raw_sha256=digest,raw_bytes=len(raw),points=len(points),
             occupied_cells=int(grid.sum()),packet_sha256=hashlib.sha256(encoded).hexdigest(),
             packet_bytes=len(encoded),grid_seconds=grid_seconds,parse_seconds=parse_seconds,
             projection_max_abs_error=error,independent_grid_exact=True)
    if 'entry' in info:row.update(archive=info['archive'],archive_entry=info['entry'],crc32=zlib.crc32(raw))
    return np.frombuffer(encoded[32:],dtype=np.uint8).copy(),row

def main():
    cfg=frozen();freeze_sha=sha(OUT/'FREEZE.json');destination=OUT/'FEATURES.json'
    assert not destination.exists()
    infos=[]
    for unit in cfg['units']:
        for pair in unit['raw_pairs']:
            for source in pair['sources']:
                infos.append(dict(source,sequence=unit['sequence'],frame=pair['frame'],index=len(infos)))
    assert len(infos)==5224
    base=OUT/'features';base.mkdir(exist_ok=True)
    job_path=base/'jobs.json'
    if job_path.exists():
        journal=json.loads(job_path.read_text());assert journal['freeze_sha256']==freeze_sha
        jobs=journal['jobs']
    else:
        local=[];missing={name:[] for name in ['test_01','test_02','test_03']}
        for info in infos:
            if (ROOT/info['path']).exists():local.append(info['index'])
            else:
                assert 'entry' in info,info['path']
                missing[info['archive']].append(info['index'])
        jobs=[dict(kind='local',indices=local[i:i+16]) for i in range(0,len(local),16)]
        for archive,indices in missing.items():
            indices.sort(key=lambda i:infos[i]['entry']['offset'])
            group=[];start=end=0
            for index in indices:
                e=infos[index]['entry'];stop=e['offset']+e['compressed']+512
                if group and (stop-start>16*1024*1024 or e['offset']-end>65536):
                    jobs.append(dict(kind='archive',archive=archive,start=start,length=end-start,indices=group));group=[]
                if not group:start=e['offset']
                group.append(index);end=stop
            if group:jobs.append(dict(kind='archive',archive=archive,start=start,length=end-start,indices=group))
        assert sorted(i for job in jobs for i in job['indices'])==list(range(5224))
        write_new(job_path,dict(freeze_sha256=freeze_sha,jobs=jobs))
    archives={name:Archive(name) for name in ['test_01','test_02','test_03']}
    for job in jobs:
        if job['kind']=='archive':assert job['start']+job['length']<=archives[job['archive']].size
    shards=base/'shards';shards.mkdir(exist_ok=True)
    def worker(number,job):
        report_path=shards/f'{number:05d}.json';values_path=shards/f'{number:05d}.npz'
        if report_path.exists():
            r=json.loads(report_path.read_text());assert r['freeze_sha256']==freeze_sha and r['job']==job
            assert r['packed_sha256']==sha(values_path)
            return r
        assert not values_path.exists(),('uncommitted feature shard',number)
        blob=None;download_seconds=0.
        if job['kind']=='archive':
            begin=time.perf_counter();blob=archives[job['archive']].read_range(job['start'],job['length'])
            download_seconds=time.perf_counter()-begin
        values=[];rows=[]
        for index in job['indices']:
            info=infos[index]
            raw=(ROOT/info['path']).read_bytes() if blob is None else extract_raw(blob,job['start'],info)
            packed,row=prepare_one(raw,info);values.append(packed);rows.append(row)
        np.savez_compressed(values_path,indices=np.array(job['indices'],dtype=np.int64),packed=np.array(values,dtype=np.uint8))
        result=dict(passed=True,freeze_sha256=freeze_sha,job=job,rows=rows,download_seconds=download_seconds,
                    range_bytes=0 if blob is None else len(blob),packed_sha256=sha(values_path))
        write_new(report_path,result)
        return result
    begin=time.perf_counter();reports=[]
    with ThreadPoolExecutor(max_workers=4) as pool:
        tasks={pool.submit(worker,index,job):index for index,job in enumerate(jobs)}
        for future in as_completed(tasks):
            reports.append(future.result())
            if len(reports)%10==0 or len(reports)==len(jobs):
                print('RAW GRID SHARDS VERIFIED',len(reports),'/',len(jobs),sum(len(r['rows']) for r in reports),'clouds',flush=True)
    arrays={u['sequence']:np.zeros((u['frames'],2,4000),dtype=np.uint8) for u in cfg['units']}
    seen=set();rows=[];artifacts={str(job_path.relative_to(ROOT)):sha(job_path)}
    for index,job in enumerate(jobs):
        report_path=shards/f'{index:05d}.json';values_path=shards/f'{index:05d}.npz'
        r=json.loads(report_path.read_text());values=np.load(values_path,allow_pickle=False)
        assert r['freeze_sha256']==freeze_sha and r['job']==job and r['packed_sha256']==sha(values_path)
        assert np.array_equal(values['indices'],job['indices'])
        for index,row,packed in zip(job['indices'],r['rows'],values['packed']):
            assert row['index']==index and index not in seen;seen.add(index);info=infos[index]
            assert (row['sequence'],row['frame'],row['source'])==(info['sequence'],info['frame'],info['source'])
            grid=np.unpackbits(packed,bitorder='little').reshape(200,160).astype(bool)
            assert row['packet_sha256']==hashlib.sha256(method.packet(grid,info['source'],info['frame'])).hexdigest()
            arrays[info['sequence']][info['frame']-1,info['source']-1]=packed;rows.append(row)
        for p in [report_path,values_path]:artifacts[str(p.relative_to(ROOT))]=sha(p)
    assert len(seen)==5224
    for name,values in arrays.items():
        p=base/f'{name}_grids.npz';assert not p.exists();np.savez_compressed(p,packed=values)
        artifacts[str(p.relative_to(ROOT))]=sha(p)
    report=dict(passed=True,freeze_sha256=freeze_sha,raw_clouds=5224,paired_frames=2612,
                rows=sorted(rows,key=lambda r:r['index']),artifacts=artifacts,
                actual_network_range_bytes=sum(r['range_bytes'] for r in reports),
                preparation_wall_seconds=time.perf_counter()-begin,
                grid_seconds=sum(r['grid_seconds'] for r in rows),raw_parse_seconds=sum(r['parse_seconds'] for r in rows),
                source_sha256=sha(Path(__file__)),raw_clouds_transmitted_in_experiment=False,
                note='Missing PCDs streamed and CRC checked; compact packet grids retained. Raw-range time is data acquisition, not algorithm compute.')
    write_new(destination,report)
    print('ALL CURRENT RAW GRIDS VERIFIED',5224,'clouds; compact packets retained',flush=True)

if __name__=='__main__':main()
