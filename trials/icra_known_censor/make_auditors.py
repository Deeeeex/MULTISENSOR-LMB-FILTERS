"""Keep old proposal audits; add the actual recursive scalar equation only."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    receipts=[]
    def write(source,name,changes,extract=None,prefix=''):
        target=OUT/name;assert not target.exists();text=source.read_text()
        if extract:text=text[text.index(extract[0]):text.index(extract[1])]
        text=prefix+text
        for before,after in changes:assert text.count(before)==1,before;text=text.replace(before,after)
        target.write_text(text)
        receipts.append(dict(source=str(source.relative_to(ROOT)),source_sha256=sha(source),target=str(target.relative_to(ROOT)),
            target_sha256=sha(target),extract=extract,prefix=prefix,changes=[dict(before=a,after=b) for a,b in changes]))
    before='    expected_r = expit((beta*logits).sum(1)+(kept*delta).sum(1)+log_i)'
    after=before+"\n    from event_audit import check_censor_event\n    expected_r = check_censor_event(run, records, expected_r, beta, logits, kept, delta, log_i)"
    write(OUT.parent/'icra_reviewer_revision/review_gaussian_audit.py','censor_gaussian_audit.py',[(before,after)])
    before='    for column, value in [(6, expected), (7, r0), (8, rER), (9, expected)]:'
    after="    from event_audit import check_censor_event\n    expected = check_censor_event(run, records, expected, b, logits, np.zeros_like(b), np.zeros_like(b), records[:, 10])\n"+before
    prefix='"""Original No-age scalar audit with explicit final-r refinement."""\nimport numpy as np\nfrom scipy.special import expit\nfrom analyze_holdout import counterfactual, score, METRICS\n\n'
    write(OUT.parent/'icra_fusion_holdout/analyze_holdout.py','censor_scalar_audit.py',[(before,after)],
        ['def audit_existence(', '\n\ndef audit_sequence('],prefix)
    write(OUT.parent/'icra_range_detection/range_audit.py','censor_range_audit.py',
        [('    from analyze_holdout import audit_existence','    from censor_scalar_audit import audit_existence')])
    (OUT/'AUDITOR_PATCHES.json').write_text(json.dumps(receipts,indent=2)+'\n')
    print('KNOWN CENSOR AUDITORS GENERATED',len(receipts),flush=True)

if __name__=='__main__':main()
