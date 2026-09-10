"""Preserve exact reuse of the complete screen and independent set scorer."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
OLD=OUT.parent/'icra_peer_detection'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    receipts=[]
    def write(name,changes):
        src=OLD/name;dst=OUT/name;assert not dst.exists();text=src.read_text()
        for a,b,count in changes:assert text.count(a)==count,a;text=text.replace(a,b)
        dst.write_text(text);receipts.append(dict(source=str(src.relative_to(ROOT)),source_sha256=sha(src),
            target=str(dst.relative_to(ROOT)),target_sha256=sha(dst),changes=[dict(before=a,after=b,count=c) for a,b,c in changes]))
    write('screen.py',[
        ('from peer_math import RULES, check_fixtures, source_state, calculate, audit_raw_weights','from direction_math import RULES, check_fixtures, source_state, calculate, audit_raw_weights',1),
        ("'peer_joint'","'nonreversal'",2)])
    source=(OLD/'verify_screen.py').read_text();start=source.index('def rebuilt(');end=source.index('\ndef extract(',start)
    write('verify_screen.py',[
        ("RULES = ['original','peer_conditional','peer_joint','no_negative']","RULES = ['original','nonreversal','negative_reversal','positive_reversal']",1),
        (source[start:end],'from independent_math import rebuilt\n\n',1),
        ("['allowed','fallback','unchanged']","['allowed','fallback','unchanged','negative_reversal','positive_reversal']",1),
        ("tolerance=1e-7 if key in ['mean','covariance'] else 1e-8 if key=='log_integral' else 2e-10 if key=='r' else 2e-14",
         "tolerance=1e-7 if key in ['mean','covariance','multiplier','kept'] else 1e-8 if key in ['log_integral','reference_log_odds','prediction_log_integral','base_direction','original_direction','final_direction'] else 2e-10 if key=='r' else 2e-14",1),
        ("'peer_joint'","'nonreversal'",4),
        ('Saved fused density minus original transmitted residual plus new residual; Cholesky integration, separate cardinality recurrence and scores',
         'Source predictions reconstructed from encoded ratios; saved density minus original residual, Brent boundary, Cholesky integration and separate cardinality and scores',1)])
    write('execute_screen.py',[
        ('RUN/ICRA_PEER_DETECTION/screen.log','RUN/ICRA_DIRECTION_CONSTRAINT/screen.log',1),
        ("    for name, digest in cfg['source_sha256'].items(): assert sha(ROOT/name) == digest, name",
         "    for name, digest in cfg['source_sha256'].items(): assert sha(ROOT/name) == digest, name\n    preflight=json.loads((OUT/'PREFLIGHT.json').read_text());assert preflight['passed'] and preflight['freeze_sha256']==sha(OUT/'SCREEN_FREEZE.json')",1),
        ("log=str(log.relative_to(ROOT)))","log=str(log.relative_to(ROOT)),preflight_sha256=sha(OUT/'PREFLIGHT.json'))",1)])
    (OUT/'PORTS.json').write_text(json.dumps(receipts,indent=2)+'\n');print('DIRECTION PORTS',len(receipts),flush=True)

if __name__=='__main__':main()
