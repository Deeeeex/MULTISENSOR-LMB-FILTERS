"""Bind a native stage to the frozen method and, when available, raw estimates."""
import argparse
import json
from common import OUT,ROOT,sha,write_new,frozen

def main():
    parser=argparse.ArgumentParser();parser.add_argument('kind',choices=['parity','corrected']);args=parser.parse_args()
    cfg=frozen();zero=args.kind=='parity';stage='alignment_'+args.kind
    path=OUT/'stages'/f'{stage}.json';assert not path.exists() and not (OUT/'results'/stage).exists()
    source={'method_freeze':sha(OUT/'FREEZE.json')}
    if not zero:
        audit=json.loads((OUT/'audit_alignment_parity.json').read_text())
        assert audit['passed'] and audit['native_runs']==4 and len(audit['parity'])==4
        for name,digest in audit['artifacts'].items():assert sha(ROOT/name)==digest,name
        estimation=json.loads((OUT/'ESTIMATION.json').read_text());assert estimation['passed']
        assert estimation['freeze_sha256']==source['method_freeze'] and estimation['paired_frames']==2612
        for name,digest in estimation['artifacts'].items():assert sha(ROOT/name)==digest,name
        source.update(alignment_estimation=sha(OUT/'ESTIMATION.json'),parity_audit=sha(OUT/'audit_alignment_parity.json'))
    units=[]
    for original in cfg['units']:
        if zero and original['sequence'] not in cfg['native_parity_sequences']:continue
        unit={k:v for k,v in original.items() if k!='raw_pairs'}
        unit['alignment_sha256']='' if zero else sha(ROOT/unit['alignment_path'])
        units.append(unit)
    assert len(units)==(2 if zero else 14)
    write_new(path,dict(protocol=cfg['protocol'],stage=stage,cohort=stage,arms=cfg['arms'],pd=cfg['pd'],
                        conditions=cfg['conditions'],alignment_mode='zero' if zero else 'occupancy',
                        units=units,source_sha256=source))
    print('NATIVE ALIGNMENT STAGE FROZEN',stage,len(units)*2,'complete trajectories',flush=True)

if __name__=='__main__':main()
