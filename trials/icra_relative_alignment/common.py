"""Provenance helpers shared by preparation and execution, not alignment math."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
ARMS=['marked_gaussian_evidence','marked_lineage']

def sha(path):
    with Path(path).open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()

def write_new(path,value):
    path=Path(path);assert not path.exists(),path
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')

def frozen():
    path=OUT/'FREEZE.json';cfg=json.loads(path.read_text())
    for name,digest in cfg['sources'].items():assert sha(ROOT/name)==digest,name
    for unit in cfg['units']:
        for field in ['data_path','marks_path','ratios_path','pose_path']:
            if unit.get(field):assert sha(ROOT/unit[field])==unit['input_files'][unit[field]],unit[field]
        for reference in unit['references'].values():assert sha(ROOT/reference['path'])==reference['sha256']
    return cfg
