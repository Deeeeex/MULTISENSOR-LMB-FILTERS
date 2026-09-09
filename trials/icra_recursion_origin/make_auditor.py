"""Preserve the established independent derivation and replace only its event branch."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    source=OUT.parent/'icra_reviewer_revision/review_gaussian_audit.py'
    destination=OUT/'intervention_gaussian_audit.py';receipt=OUT/'AUDITOR_PATCH.json'
    assert not destination.exists() and not receipt.exists()
    before="    if run['arm'] == 'marked_gaussian_evidence_guarded_scalar':\n        mean, cov, log_i = base_mean, base_cov, records[:, 10].copy()"
    after="    if run['arm'] == 'marked_gaussian_evidence_guarded_scalar':\n        from event_audit import apply_expected_event\n        mean, cov, log_i = apply_expected_event(run, records, base_mean, base_cov, mean, cov, log_i, beta, logits, kept, delta)"
    content=source.read_text();assert content.count(before)==1
    destination.write_text(content.replace(before,after))
    receipt.write_text(json.dumps(dict(source=str(source.relative_to(ROOT)),source_sha256=sha(source),
        output_sha256=sha(destination),changes=[dict(before=before,after=after)]),indent=2)+'\n')
    print('INDEPENDENT GAUSSIAN AUDITOR GENERATED',flush=True)

if __name__=='__main__':main()
