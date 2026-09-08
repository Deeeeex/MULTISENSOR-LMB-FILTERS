"""Only prepare the complete ID-defined holdout, without tracking outcomes."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import importlib.util,json,hashlib
import numpy as np
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
spec=importlib.util.spec_from_file_location('previous_transform_reader',OUT.parent/'icra_method_iteration/fetch_transfer_transforms.py')
previous=importlib.util.module_from_spec(spec);spec.loader.exec_module(previous)
SELECT=[s for s in range(32) if s not in range(0,32,5)]


def main():
    previous.CACHE.mkdir(parents=True,exist_ok=True)
    source=ROOT/'tmp/external_baselines/DMSTrack/AB3DMOT/scripts/KITTI/v2v4real_train_evaluate_tracking.seqmap.val'
    lengths=[int(line.split()[3])+1 for line in source.read_text().splitlines()]
    offsets=np.r_[0,np.cumsum(lengths)]
    selected={i for seq in SELECT for i in range(offsets[seq],offsets[seq+1])}
    entries=previous.directory();assert len(entries)==2*sum(lengths)
    chosen=[e for e in entries if int(e['name'].split('/')[-1].split('_')[0]) in selected]
    assert len(chosen)==11202 and len(SELECT)==25 and len(selected)==5601
    hashes=[]
    with ThreadPoolExecutor(max_workers=8) as pool:
        for future in as_completed([pool.submit(previous.reader.fetch,e) for e in chosen]):
            hashes.append(future.result())
            if len(hashes)%500==0:print('Verified remaining-cohort transforms',len(hashes),'/',len(chosen),flush=True)
    report=dict(source=previous.reader.URL,archive_size=previous.reader.SIZE,selected_sequences=SELECT,
                complete_train_lengths=lengths,matrix_count=len(hashes),frames=5601,
                seqmap_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                central_sha256=(previous.CACHE/'central_sha256.txt').read_text().strip(),
                source_files=sorted(hashes,key=lambda x:(x['sensor'],x['frame'])))
    (OUT/'transform_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print('REMAINING COHORT TRANSFORMS COMPLETE',len(hashes),'CRC/hash verified matrices; no tracking outcomes.',flush=True)


if __name__=='__main__':main()
