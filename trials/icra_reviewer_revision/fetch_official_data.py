"""Read the public UCLA release, with ZIP directory/CRC and content hashes.

The public Box page grants anonymous download access. Follow that page's
normal scoped-read-token flow; no user login/credentials are read or stored.
Ephemeral read tokens and signed download URLs stay in process memory and
are never printed or written into provenance files.
"""
from concurrent.futures import ThreadPoolExecutor,as_completed
from pathlib import Path,PurePosixPath
import argparse
import hashlib
import http.cookiejar
import json
import struct
import threading
import time
import urllib.error
import urllib.request
import zlib

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
CACHE=ROOT/'tmp/external_baselines/v2v_official'
SHARED='6w5qgaynlv2vj9dii435mqbnucz91tx7'
VANITY='https://ucla.app.box.com/v/UCLA-MobilityLab-V2V4REAL'
# Discovered in the public Data folder 279924274808, not guessed file IDs.
ARCHIVES={
 'val':('1619910191481',1045857068),
 'test_01':('1619923463037',633582640),
 'test_02':('1619910080470',1228581180),
 'test_03':('1619915891150',1551328836),
 'train_01':('1619935662984',2119374798),
 'train_02':('1619925738887',950136211),
 'train_03':('1619916750660',1671705484),
 'train_04':('1619912061459',832603449),
 'train_05':('1619923727837',1451830950),
 'train_06':('1619904875978',1709593898),
 'train_07':('1619910858734',1347978238),
 'train_08':('1619910258966',851238796),
}


def parse_object(text,marker):
    return json.JSONDecoder().raw_decode(text.split(marker,1)[1])[0]


class Archive:
    def __init__(self,name):
        self.name=name;self.file_id,self.size=ARCHIVES[name]
        self.url=None;self.lock=threading.Lock()
        self.cache=CACHE/name;self.cache.mkdir(parents=True,exist_ok=True)

    def refresh(self):
        opener=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
        page=f'https://ucla.app.box.com/s/{SHARED}/file/{self.file_id}'
        with opener.open(page,timeout=35) as response:html=response.read().decode()
        preview=parse_object(html,'Box.prefetchedData = ')['preview_metadata']
        assert preview['permissions']['can_download'] and preview['is_download_available']
        assert preview['size']==self.size and preview['name']==self.name+'.zip'
        config=parse_object(html,'Box.config = ')
        request=urllib.request.Request('https://app.box.com/app-api/enduserapp/elements/tokens',
            data=json.dumps({'fileIDs':[self.file_id]}).encode(),headers={
                'Content-Type':'application/json','X-Request-Token':config['requestToken'],
                'X-Box-EndUser-API':'sharedName='+SHARED})
        with opener.open(request,timeout=35) as response:token=json.load(response)[self.file_id]['read']
        request=urllib.request.Request('https://api.box.com/2.0/files/'+self.file_id+'/content',
            headers={'Authorization':'Bearer '+token,'BoxApi':'shared_link=https://app.box.com/s/'+SHARED,
                     'Range':'bytes=0-0'})
        with opener.open(request,timeout=35) as response:
            assert response.status==206 and response.headers['Content-Range']==f'bytes 0-0/{self.size}'
            self.url=response.geturl();assert response.read(2)==b'P'

    def read_range(self,start,length):
        assert 0<=start<self.size and 0<length<=self.size-start
        if self.url is None:
            with self.lock:
                if self.url is None:self.refresh()
        for attempt in range(5):
            current=self.url
            try:
                request=urllib.request.Request(current,headers={'Range':f'bytes={start}-{start+length-1}'})
                with urllib.request.urlopen(request,timeout=45) as response:
                    assert response.status==206
                    assert response.headers['Content-Range']==f'bytes {start}-{start+length-1}/{self.size}'
                    raw=response.read(length+1)
                assert len(raw)==length
                return raw
            except Exception as error:
                code=getattr(error,'code',None)
                if attempt==4:raise RuntimeError(f'Public Box range failed: {self.name} offset={start} HTTP={code}') from None
                if code in [401,403]:
                    with self.lock:
                        if self.url==current:self.refresh()
                time.sleep(attempt+1)

    def directory(self):
        path=self.cache/'directory.json'
        if path.exists():return json.loads(path.read_text())
        tail=self.read_range(self.size-65536,65536)
        end=tail.rfind(b'PK\x05\x06');assert end>=0
        fields=struct.unpack_from('<4s4H2IH',tail,end)
        count,central_size,central_offset=fields[4],fields[5],fields[6]
        assert fields[1]==fields[2]==0
        if count==65535 or central_size==0xffffffff or central_offset==0xffffffff:
            at=tail.rfind(b'PK\x06\x06');assert at>=0
            fields64=struct.unpack_from('<4sQHHIIQQQQ',tail,at)
            count,central_size,central_offset=fields64[-3:]
        central=self.read_range(central_offset,central_size)
        entries=[];cursor=0
        while cursor<len(central):
            v=struct.unpack_from('<4s6H3I5H2I',central,cursor);assert v[0]==b'PK\x01\x02'
            name=central[cursor+46:cursor+46+v[10]].decode('utf-8')
            assert not PurePosixPath(name).is_absolute() and '..' not in PurePosixPath(name).parts
            extra=central[cursor+46+v[10]:cursor+46+v[10]+v[11]]
            offset,compressed,raw=v[-1],v[8],v[9];at=0
            while at+4<=len(extra):
                tag,length=struct.unpack_from('<HH',extra,at);payload=extra[at+4:at+4+length];at+=4+length
                if tag==1:
                    values=list(struct.unpack('<'+'Q'*(len(payload)//8),payload))
                    if raw==0xffffffff:raw=values.pop(0)
                    if compressed==0xffffffff:compressed=values.pop(0)
                    if offset==0xffffffff:offset=values.pop(0)
            entries.append(dict(name=name,offset=offset,compressed=compressed,raw=raw,compression=v[4],crc=v[7]))
            cursor+=46+v[10]+v[11]+v[12]
        assert len(entries)==count
        result=dict(name=self.name,file_id=self.file_id,archive_bytes=self.size,
                    source=VANITY+'/file/'+self.file_id,central_sha256=hashlib.sha256(central).hexdigest(),entries=entries)
        path.write_text(json.dumps(result,indent=2)+'\n')
        print('DIRECTORY',self.name,len(entries),'entries',central_size,'bytes',flush=True)
        return result

    def read_file(self,entry):
        target=self.cache/'files'/entry['name'];target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists():raw=target.read_bytes()
        else:
            length=min(entry['compressed']+512,self.size-entry['offset'])
            first=self.read_range(entry['offset'],length)
            v=struct.unpack_from('<4s5H3I2H',first);assert v[0]==b'PK\x03\x04'
            start=30+v[-2]+v[-1];stop=start+entry['compressed']
            payload=first[start:stop] if stop<=len(first) else self.read_range(entry['offset']+start,entry['compressed'])
            assert entry['compression'] in [0,8]
            raw=zlib.decompress(payload,-15) if entry['compression']==8 else payload
            assert len(raw)==entry['raw'] and zlib.crc32(raw)==entry['crc']
            temporary=target.with_name(target.name+'.part');temporary.write_bytes(raw);temporary.replace(target)
        assert len(raw)==entry['raw'] and zlib.crc32(raw)==entry['crc']
        return dict(name=entry['name'],path=str(target.relative_to(ROOT)),bytes=len(raw),
                    crc32=entry['crc'],sha256=hashlib.sha256(raw).hexdigest())


def main():
    p=argparse.ArgumentParser();p.add_argument('archives',nargs='+',choices=list(ARCHIVES))
    p.add_argument('--kind',choices=['directory','yaml','pcd'],default='directory')
    p.add_argument('--workers',type=int,default=12);args=p.parse_args()
    for name in args.archives:
        archive=Archive(name);directory=archive.directory()
        if args.kind=='directory':
            groups=sorted({str(PurePosixPath(e['name']).parent) for e in directory['entries'] if e['name'].endswith('.yaml')})
            print('YAML GROUPS',name,groups,flush=True);continue
        entries=[e for e in directory['entries'] if e['name'].endswith('.'+args.kind) and '/__MACOSX/' not in e['name']]
        records=[]
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            for future in as_completed([pool.submit(archive.read_file,e) for e in entries]):
                records.append(future.result())
                if len(records)%100==0:print('FETCHED',name,args.kind,len(records),'/',len(entries),flush=True)
        dest=archive.cache/f'{args.kind}_manifest.json'
        result=dict(source=directory['source'],archive_bytes=archive.size,
                    directory_sha256=hashlib.sha256((archive.cache/'directory.json').read_bytes()).hexdigest(),
                    kind=args.kind,count=len(records),files=sorted(records,key=lambda r:r['name']))
        dest.write_text(json.dumps(result,indent=2)+'\n')
        print('COMPLETE PUBLIC DATA',name,args.kind,len(records),'CRC-checked files',flush=True)


if __name__=='__main__':main()
