"""Reuse the checked external adapter with bounded path/count-only changes."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
OLD=OUT.parent/'icra_v2x_transfer'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
rows=[]


def copy(name, changes=()):
    source,destination=OLD/name,OUT/name
    assert not destination.exists()
    value=source.read_text()
    for old,new in changes:
        assert value.count(old)==1,(name,old,value.count(old))
        value=value.replace(old,new)
    destination.write_text(value)
    rows.append(dict(source=str(source),source_sha256=sha(source),destination=name,
        destination_sha256=sha(destination),replacements=[dict(before=a,after=b) for a,b in changes]))


copy('input_adapter.py')
copy('check_adapter.py')
copy('prepare_inputs.py')
copy('audit_inputs.py')
copy('infer_transfer.py',[("619, 'paired frames'","2172, 'paired frames'")])
copy('freeze_inference.py',[
    ("'check_adapter.py', 'ADAPTER_CHECK.json', 'ADAPTER_NOTES.md',\n                              'SHARED_ANNOTATION_DIAGNOSTIC.json'",
     "'check_adapter.py', 'ADAPTER_CHECK.json', 'INPUT_PIPELINE_PATCH.json', 'fetch_inputs.py',\n                              'prepare_inputs.py', 'audit_inputs.py'"),
    ("paths += [old.OUT / 'official_input_adapter.py']",
     "paths += [old.OUT / 'official_input_adapter.py', OUT.parent / 'icra_v2x_transfer/ADAPTER_NOTES.md',\n              OUT.parent / 'icra_v2x_transfer/SHARED_ANNOTATION_DIAGNOSTIC.json']"),
    ('paired_frames=619','paired_frames=2172')])
(OUT/'.gitignore').write_text('detections/\nnew_data/\npose_inputs/\nresults/\n__pycache__/\n')
destination=OUT/'INPUT_PIPELINE_PATCH.json';assert not destination.exists()
destination.write_text(json.dumps(dict(files=rows,generator_sha256=sha(Path(__file__))),indent=2)+'\n')
print('ADDITIONAL TEST PIPELINE PREPARED; ACQUISITION STILL REQUIRES THE COMPLETE DEVELOPMENT GATE')
