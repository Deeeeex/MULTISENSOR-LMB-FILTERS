"""Construct this registered runner once, retaining the immutable first round."""
from pathlib import Path
import hashlib
import json

out=Path(__file__).resolve().parent;root=out.parents[1]
old=out.parent/'icra_method_iteration'
s=(old/'runMethodReplayFast.m').read_text()
s=s.replace('function runMethodReplayFast(', 'function runJointEvidenceReplay(')
s=s.replace("arms={'qualified_exist','innovation_recency'}", "arms={'qualified_exist','joint_evidence','joint_evidence_recency'}")
s=s.replace("original=fullfile(root,'trials','icra_external_fusion');", "original=fullfile(root,'trials','icra_external_fusion');\npriorIteration=fullfile(root,'trials','icra_method_iteration');addpath(priorIteration);\ncheckJointEvidence();")
s=s.replace("runtimePath=fullfile(out,'runtime')", "runtimePath=fullfile(priorIteration,'runtime')")
s=s.replace("'results_v2v_fast'", "'results_development'")
s=s.replace("'source_sha256_ir_fast.json'", "'source_sha256.json'")
s=s.replace("'current-innovation-recency-v1','implementation','ir-v1-accelerated'", "'joint-current-evidence-v1','implementation','je-v1'")
s=s.replace('fuseInnovationRecency(inputs', 'fuseJointEvidence(inputs')
(out/'runJointEvidenceReplay.m').write_text(s)
base=json.loads((old/'source_sha256_ir_fast.json').read_text())
for name,digest in base.items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
for p in list(out.glob('*.m'))+[out/'PROTOCOL.md',out/'make_replay.py']:
    base[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
for name in ['summary_development.json','summary_conservative.json','summary_confirmed.json','summary_transfer.json']:
    p=old/name;base[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
(out/'source_sha256.json').write_text(json.dumps(base,indent=2,sort_keys=True)+'\n')
print('JE sources frozen',len(base),'files; earlier snapshots unchanged.')
