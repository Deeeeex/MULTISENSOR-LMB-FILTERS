"""Range-download only reserved-cohort transforms from the public train ZIP."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import importlib.util,json,hashlib,struct
import numpy as np
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
spec=importlib.util.spec_from_file_location('frozen_zip_reader',OUT.parent/'icra_external_fusion/fetch_v2v_transforms.py')
reader=importlib.util.module_from_spec(spec);spec.loader.exec_module(reader)
reader.SIZE=19872259356
reader.URL='https://drive.usercontent.google.com/download?id=1ZF8fkCriZ4JRX5eopSGJypuj90thswj4&export=download&confirm=t'
reader.CACHE=ROOT/'tmp/external_baselines/v2v_transfer_transforms'
CACHE=reader.CACHE
SELECT=list(range(0,32,5))

def directory():
    saved=CACHE/'directory.json'
    if saved.exists():return json.loads(saved.read_text())
    tail=reader.read_range(reader.SIZE-65536,65536)
    index=tail.rfind(b'PK\x06\x06');assert index>=0
    fields=struct.unpack_from('<4sQHHIIQQQQ',tail,index)
    central=reader.read_range(fields[-1],fields[-2]);entries=[];cursor=0
    while cursor<len(central):
        v=struct.unpack_from('<4s6H3I5H2I',central,cursor);assert v[0]==b'PK\x01\x02'
        name=central[cursor+46:cursor+46+v[10]].decode()
        extra=central[cursor+46+v[10]:cursor+46+v[10]+v[11]]
        offset,compressed,raw=v[-1],v[8],v[9];at=0
        while at+4<=len(extra):
            tag,length=struct.unpack_from('<HH',extra,at);payload=extra[at+4:at+4+length];at+=4+length
            if tag==1:
                values=list(struct.unpack('<'+'Q'*(len(payload)//8),payload))
                if raw==0xffffffff:raw=values.pop(0)
                if compressed==0xffffffff:compressed=values.pop(0)
                if offset==0xffffffff:offset=values.pop(0)
        if name.endswith('_transformation_matrix.npy'):
            entries.append(dict(name=name,offset=offset,compressed=compressed,raw=raw,compression=v[4],crc=v[7]))
        cursor+=46+v[10]+v[11]+v[12]
    saved.write_text(json.dumps(entries,indent=2)+'\n')
    (CACHE/'central_sha256.txt').write_text(hashlib.sha256(central).hexdigest()+'\n')
    return entries

def main():
    CACHE.mkdir(parents=True,exist_ok=True)
    source=ROOT/'tmp/external_baselines/DMSTrack/AB3DMOT/scripts/KITTI/v2v4real_train_evaluate_tracking.seqmap.val'
    lengths=[int(line.split()[3])+1 for line in source.read_text().splitlines()]
    offsets=np.r_[0,np.cumsum(lengths)];selected={i for seq in SELECT for i in range(offsets[seq],offsets[seq+1])}
    entries=directory();assert len(entries)==2*sum(lengths)
    chosen=[e for e in entries if int(e['name'].split('/')[-1].split('_')[0]) in selected]
    assert len(chosen)==3008
    hashes=[]
    with ThreadPoolExecutor(max_workers=8) as pool:
        for future in as_completed([pool.submit(reader.fetch,e) for e in chosen]):
            hashes.append(future.result())
            if len(hashes)%200==0:print('Verified reserved transforms',len(hashes),'/',len(chosen),flush=True)
    report=dict(source=reader.URL,archive_size=reader.SIZE,selected_sequences=SELECT,
                complete_train_lengths=lengths,matrix_count=len(hashes),frames=1504,
                seqmap_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                central_sha256=(CACHE/'central_sha256.txt').read_text().strip(),
                source_files=sorted(hashes,key=lambda x:(x['sensor'],x['frame'])))
    (OUT/'transfer_transform_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print('TRANSFER TRANSFORMS COMPLETE',len(hashes),'CRC/hash verified matrices; no tracking outcomes.',flush=True)

if __name__=='__main__':main()
