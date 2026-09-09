"""Register sources and every input before a new immutable tracking stage."""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
OLD = OUT.parent / 'icra_gaussian_evidence'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def register(stage, cohort, arms, pd=.9, preflight=False):
    path = OUT / 'stages' / f'{stage}.json'
    assert not path.exists() and not (OUT / 'results' / stage).exists()
    source = json.loads((OLD / 'source_sha256.json').read_text())
    for name, expected in source.items():
        assert sha(ROOT / name) == expected, name
    frozen = OUT / 'PRE_REVISION_FREEZE.json'
    if not frozen.exists():
        paths = [OLD / 'ROUND_FREEZE.json', OLD / 'source_sha256.json',
                 OUT.parent / 'icra_ceiling_iteration/calibration.json',
                 OUT.parent / 'icra_marked_iteration/likelihood_manifest.json',
                 ROOT / 'papers/icra2027/main.tex',
                 ROOT / 'tmp/external_baselines/DMSTrack/V2V4Real/official_models/no_fusion_keep_all/net_epoch60.pth',
                 ROOT / 'tmp/external_baselines/DMSTrack/V2V4Real/official_models/no_fusion_keep_all/config.yaml']
        frozen.write_text(json.dumps(dict(base_commit='9479f0d39958056464ab23ec07da94244abb5711',
            created_utc=datetime.now(timezone.utc).isoformat(),
            prior_sources=source, frozen_artifacts={str(p.relative_to(ROOT)):sha(p) for p in paths},
            exposed_sequence_groups={'dmstrack_val':list(range(9)), 'dmstrack_train':list(range(32))},
            rule='GCE and old score calibration remain frozen regardless of new results.'), indent=2)+'\n')
    protected = [OUT / n for n in ['PROTOCOL.md', 'PRE_REVISION_FREEZE.json', 'make_extension.py',
                 'checkReviewerEvidence.m', 'fuseReviewerEvidence.m', 'runReviewerReplay.m',
                 'register_stage.py', 'run_stage.py']]
    source.update({str(p.relative_to(ROOT)):sha(p) for p in protected})
    transfer = cohort == 'seen_transfer'
    manifest_path = OUT.parent / ('icra_fusion_holdout/input_manifest.json' if transfer
                                  else 'icra_external_fusion/v2v4real_input_manifest.json')
    manifest = json.loads(manifest_path.read_text())
    units = []
    for row in manifest['sequences']:
        seq = int(row['sequence']); name = f'{seq:04d}'
        data_path = OUT.parent / ('icra_fusion_holdout' if transfer else 'icra_external_fusion') / 'data' / f'v2v4real_{name}.mat'
        assert sha(data_path) == row['input_sha256'], str(data_path)
        unit = dict(sequence=name, data_path=str(data_path.relative_to(ROOT)),
                    input_sha256=sha(data_path), radio_seed=8301+seq)
        source[unit['data_path']] = unit['input_sha256']
        if not transfer:
            for field, folder, filename in [('marks_path','icra_ceiling_iteration/data_marks',f'marks_{name}.mat'),
                                            ('ratios_path','icra_marked_iteration/data_likelihoods',f'likelihoods_{name}.mat')]:
                p = OUT.parent / folder / filename
                unit[field] = str(p.relative_to(ROOT)); source[unit[field]] = sha(p)
        units.append(unit)
    if preflight:
        assert not transfer
        units = units[:1]
    cfg = dict(protocol='icra-reviewer-revision-v1', stage=stage, cohort=cohort,
               created_utc=datetime.now(timezone.utc).isoformat(), arms=arms, pd=pd,
               preflight=preflight, conditions=['reliable','intermittent'],
               units=units, source_sha256=source)
    path.parent.mkdir(exist_ok=True)
    path.write_text(json.dumps(cfg,indent=2,allow_nan=False)+'\n')
    print('REGISTERED',stage,len(units),'sequences',len(arms),'arms',2*len(units)*len(arms),'outputs',path)
    return path


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--stage',required=True)
    p.add_argument('--cohort',choices=['development','seen_transfer'],required=True)
    p.add_argument('--arms',nargs='+',required=True)
    p.add_argument('--pd',type=float,default=.9)
    p.add_argument('--preflight',action='store_true')
    args = p.parse_args()
    register(args.stage,args.cohort,args.arms,args.pd,args.preflight)
