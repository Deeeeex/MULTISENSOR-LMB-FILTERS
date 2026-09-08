"""Read only coordinate-transform NPYs from the authors' 6.3 GB public ZIP.

HTTP byte ranges, ZIP64 offsets and CRC32 are checked. No model features,
images or point clouds are downloaded. NumPy loading disables pickle.
"""
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
import hashlib
import io
import json
import struct
import time
import urllib.request
import zlib
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
CACHE=ROOT/'tmp/external_baselines/v2v_transforms'
SIZE=6324129277
URL=('https://drive.usercontent.google.com/download?'
     'id=1RCxGMm9tSB94CHMyVOu4dhZfRFsnNpUf&export=download&confirm=t')


def read_range(start, length):
    expected=f'bytes {start}-{start+length-1}/{SIZE}'
    for attempt in range(4):
        try:
            req=urllib.request.Request(URL,headers={'Range':f'bytes={start}-{start+length-1}'})
            with urllib.request.urlopen(req,timeout=40) as response:
                assert response.status==206 and response.headers['Content-Range']==expected
                content=response.read(length+1)
            assert len(content)==length
            return content
        except Exception:
            if attempt==3: raise
            time.sleep(attempt+1)


def directory():
    saved=CACHE/'directory.json'
    if saved.exists(): return json.loads(saved.read_text())
    tail=read_range(SIZE-65536,65536)
    index=tail.rfind(b'PK\x06\x06'); assert index>=0
    fields=struct.unpack_from('<4sQHHIIQQQQ',tail,index)
    central=read_range(fields[-1],fields[-2]); entries=[]; cursor=0
    while cursor<len(central):
        v=struct.unpack_from('<4s6H3I5H2I',central,cursor); assert v[0]==b'PK\x01\x02'
        name=central[cursor+46:cursor+46+v[10]].decode()
        extra=central[cursor+46+v[10]:cursor+46+v[10]+v[11]]
        offset,compressed,raw=v[-1],v[8],v[9]; at=0
        while at+4<=len(extra):
            tag,length=struct.unpack_from('<HH',extra,at)
            payload=extra[at+4:at+4+length]; at+=4+length
            if tag==1:
                values=list(struct.unpack('<'+'Q'*(len(payload)//8),payload))
                if raw==0xffffffff: raw=values.pop(0)
                if compressed==0xffffffff: compressed=values.pop(0)
                if offset==0xffffffff: offset=values.pop(0)
        if name.endswith('_transformation_matrix.npy'):
            entries.append(dict(name=name,offset=offset,compressed=compressed,raw=raw,
                                compression=v[4],crc=v[7]))
        cursor+=46+v[10]+v[11]+v[12]
    assert len(entries)==3986
    saved.write_text(json.dumps(entries,indent=2)+'\n')
    (CACHE/'central_sha256.txt').write_text(hashlib.sha256(central).hexdigest()+'\n')
    return entries


def fetch(entry):
    parts=entry['name'].split('/'); sensor=parts[-2]; name=parts[-1]
    assert sensor in ['ego','1'] and name[:4].isdigit()
    target=CACHE/sensor/name; target.parent.mkdir(exist_ok=True)
    if target.exists():
        raw=target.read_bytes()
    else:
        # ZIP local extra fields are bounded at 65535 B. Inspect 256 B first;
        # a second exact range is needed only if the payload is outside it.
        first=read_range(entry['offset'],max(256,entry['compressed']+128))
        v=struct.unpack_from('<4s5H3I2H',first); assert v[0]==b'PK\x03\x04'
        start=30+v[-2]+v[-1]; stop=start+entry['compressed']
        payload=first[start:stop] if stop<=len(first) else read_range(entry['offset']+start,entry['compressed'])
        assert entry['compression']==8
        raw=zlib.decompress(payload,-15)
        assert len(raw)==entry['raw'] and zlib.crc32(raw)==entry['crc']
        target.write_bytes(raw)
    assert len(raw)==entry['raw'] and zlib.crc32(raw)==entry['crc']
    matrix=np.load(io.BytesIO(raw),allow_pickle=False)
    assert matrix.shape==(4,4) and np.isfinite(matrix).all()
    assert np.allclose(matrix[3],[0,0,0,1],atol=1e-5)
    return dict(sensor=sensor,frame=int(name[:4]),sha256=hashlib.sha256(raw).hexdigest())


def main():
    CACHE.mkdir(parents=True,exist_ok=True); entries=directory(); hashes=[]
    with ThreadPoolExecutor(max_workers=8) as pool:
        futures={pool.submit(fetch,e):e for e in entries}
        for future in as_completed(futures):
            hashes.append(future.result())
            if len(hashes)%200==0: print('Verified transforms',len(hashes),'/',len(entries),flush=True)
    arrays={sensor:np.stack([np.load(CACHE/sensor/f'{i:04d}_transformation_matrix.npy',allow_pickle=False)
                            for i in range(1993)]) for sensor in ['ego','1']}
    np.savez_compressed(CACHE/'transforms.npz',**arrays)
    report=dict(source=URL,archive_size=SIZE,matrix_count=len(hashes),frames_per_sensor=1993,
                central_sha256=(CACHE/'central_sha256.txt').read_text().strip(),
                source_files=sorted(hashes,key=lambda r:(r['sensor'],r['frame'])))
    (CACHE/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print('COMPLETED: 3986 CRC-checked source transforms; no feature archive download.',flush=True)


if __name__=='__main__':main()
