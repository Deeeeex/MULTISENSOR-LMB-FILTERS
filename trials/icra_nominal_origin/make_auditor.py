"""Keep the established Gaussian reconstruction and add only the scalar event equation."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    source=OUT.parent/'icra_reviewer_revision/review_gaussian_audit.py';target=OUT/'initial_gaussian_audit.py';assert not target.exists()
    before="    expected_r = expit((beta*logits).sum(1)+(kept*delta).sum(1)+log_i)"
    after=before+"\n    if run['initialNegativeEnabled']:\n        from event_audit import check_initial_event\n        expected_r = check_initial_event(run, records, expected_r, beta, logits, kept, delta, log_i)"
    text=source.read_text();assert text.count(before)==1;target.write_text(text.replace(before,after))
    (OUT/'AUDITOR_PATCH.json').write_text(json.dumps(dict(source=str(source.relative_to(ROOT)),source_sha256=sha(source),
        target=str(target.relative_to(ROOT)),target_sha256=sha(target),changes=[dict(before=before,after=after)]),indent=2)+'\n')
    print('INITIAL SCALAR EVENT AUDITOR GENERATED',flush=True)

if __name__=='__main__':main()
