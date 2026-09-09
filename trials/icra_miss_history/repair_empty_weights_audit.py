"""Retain the failed auditor and handle the native empty-measurement W."""
from pathlib import Path
import hashlib
import json
import py_compile

OUT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
changes=[]
for oldname,newname,before,after in [
 ('history_audit.py','history_audit_v2.py',
  "            if not len(rows):assert not raw.size;continue\n            W=raw.reshape(len(rows),len(marks)+1)",
  "            if not len(rows):assert not raw.size;continue\n            if not raw.size:\n                assert not rows[:,8:10].any(),'empty native weights give zero positive support and association mass'\n                checked+=len(rows)\n                continue\n            W=raw.reshape(len(rows),len(marks)+1)"),
 ('audit_stage.py','audit_stage_v2.py',
  'from history_audit import audit_positive_marks,audit_matching',
  'from history_audit_v2 import audit_positive_marks,audit_matching')]:
    source=OUT/oldname;destination=OUT/newname;text=source.read_text()
    assert text.count(before)==1 and not destination.exists()
    destination.write_text(text.replace(before,after));py_compile.compile(str(destination),doraise=True)
    changes.append(dict(source=oldname,source_sha256=sha(source),destination=newname,destination_sha256=sha(destination),before=before,after=after))
receipt=dict(failure='ValueError: cannot reshape array of size 0 into shape (7,1)',
             context='First early audit of 0000 reliable GCE after the density check passed; native code records W=[] for an empty measurement set.',
             fix='Check both positive support and association mass are zero when the native W log is empty.',
             changes=changes,algorithm_or_selection_changed=False)
(OUT/'EMPTY_WEIGHTS_AUDIT_REPAIR.json').write_text(json.dumps(receipt,indent=2)+'\n')
print('PRESERVED INITIAL AUDITORS; CREATED V2 FOR EMPTY NATIVE WEIGHTS')
