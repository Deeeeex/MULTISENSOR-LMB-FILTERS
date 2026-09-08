"""Audit source identity before interpreting reserved-sequence tracking scores."""
from pathlib import Path
import collections,hashlib,json
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[1]
BASE=ROOT/'tmp/external_baselines/DMSTrack/AB3DMOT/scripts/KITTI'

def frames(path):
    groups=collections.defaultdict(list)
    for line in path.read_text().splitlines():
        row=line.split()
        groups[int(row[0])].append(tuple(round(float(row[k]),5) for k in [10,11,12,13,14,15,16]))
    return {t:tuple(sorted(rows)) for t,rows in groups.items()}

def main():
    val={p.stem:hashlib.sha256(p.read_bytes()).hexdigest() for p in (BASE/'v2v4real_val_label').glob('*.txt')}
    fingerprints={};hashes={};records=[]
    for path in sorted((BASE/'v2v4real_val_label').glob('*.txt')):
        hashes[str(path.relative_to(BASE))]=hashlib.sha256(path.read_bytes()).hexdigest()
        for t,f in frames(path).items():fingerprints.setdefault(f,[]).append((path.stem,t))
    for path in sorted((BASE/'v2v4real_train_label').glob('*.txt')):
        digest=hashlib.sha256(path.read_bytes()).hexdigest();hashes[str(path.relative_to(BASE))]=digest
        f=frames(path);overlap=[dict(train_frame=t,val_matches=fingerprints[fp]) for t,fp in f.items() if fp in fingerprints]
        same=[k for k,v in val.items() if v==digest]
        records.append(dict(sequence=path.stem,entire_label_duplicates=same,nonempty_frames=len(f),
                            overlap_frame_count=len(overlap),overlap=overlap))
    assert len(records)==32 and len(val)==9
    report=dict(type='pre-tracking exact geometry duplication audit',position_precision_decimals=5,
                fields='sorted KITTI dimensions, position, yaw; IDs ignored',source_files=hashes,train_sequences=records)
    (OUT/'transfer_overlap_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Overlap audit:',[(r['sequence'],r['overlap_frame_count']) for r in records if r['overlap_frame_count']])

if __name__=='__main__':main()
