"""Continue the frozen sequence only after complete native preflight and each audit."""
from pathlib import Path
import hashlib
import json
import subprocess
import time

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
PYTHON=ROOT/'tmp/external_baselines/v2v_inference_venv/bin/python'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    ledger=OUT/'PIPELINE_EXECUTION.json';assert not ledger.exists()
    scripts=['audit_stage_v2.py','summarize.py','run_stage.py','verify_selection.py','build_report.py']
    receipt=dict(source_sha256={s:sha(OUT/s) for s in scripts},orchestrator_sha256=sha(Path(__file__)),steps=[])
    ledger.write_text(json.dumps(receipt,indent=2)+'\n')
    native=OUT/'runtime_range_detection_preflight.json'
    while True:
        if native.exists():
            rows=json.loads(native.read_text())
            if len(rows)==2:
                assert all(r['returncode']==0 and r['completion_line'] and r['files']==18 for r in rows)
                break
        time.sleep(1)
    commands=[
        [str(PYTHON),'-u',str(OUT/'audit_stage_v2.py'),'range_detection_preflight'],
        [str(PYTHON),'-u',str(OUT/'summarize.py'),'--preflight-only'],
        ['python3','-u',str(OUT/'run_stage.py'),'range_detection_screen','--workers','2'],
        [str(PYTHON),'-u',str(OUT/'audit_stage_v2.py'),'range_detection_screen'],
        [str(PYTHON),'-u',str(OUT/'summarize.py')],
        [str(PYTHON),'-u',str(OUT/'verify_selection.py')],
        [str(PYTHON),'-u',str(OUT/'build_report.py')],
    ]
    for command in commands:
        for s,expected in receipt['source_sha256'].items():assert sha(OUT/s)==expected,s
        print('PIPELINE START',command[2:],flush=True)
        process=subprocess.run(command,cwd=ROOT)
        receipt['steps'].append(dict(command=command,returncode=process.returncode))
        ledger.write_text(json.dumps(receipt,indent=2)+'\n')
        assert process.returncode==0,command
        print('PIPELINE FINISHED',command[2:],flush=True)
    receipt['complete']=True;ledger.write_text(json.dumps(receipt,indent=2)+'\n')
    print('COMPLETE FROZEN RANGE DETECTION PIPELINE',flush=True)

if __name__=='__main__':main()
