"""Continue the registered test audit and summary after its native stage exits."""
from pathlib import Path
import json
import subprocess
import sys
import time

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
LOG=ROOT/'RUN/ICRA_TEMPORAL_ASSOCIATION'
STAGE='association_screen_selected_test'


def main():
    initial=json.loads((OUT/('runtime_'+STAGE+'.json')).read_text())
    assert len(initial) in [13,14]
    for row in initial:
        assert row['returncode']==0 and row['completion_line'] and row['files']==10
    assert len(list((OUT/'partial_audits'/STAGE).glob('*.json')))==13
    assert not (OUT/'partial_audits'/STAGE/'v2xt_0001.json').exists()
    print('WAITING FOR FINAL NATIVE UNIT',flush=True)
    while True:
        runtime=json.loads((OUT/('runtime_'+STAGE+'.json')).read_text())
        assert all(r['returncode']==0 and r['completion_line'] and r['files']==10 for r in runtime)
        marker='ALL REVIEW STAGE RUNS COMPLETE '+STAGE+' 14 sequences'
        if len(runtime)==14 and marker in (LOG/'additional_test_native.log').read_text():break
        time.sleep(5)
    assert len(list((OUT/'results'/STAGE).glob('*.json.gz')))==140
    steps=[
        ('audit_unit_v2xt_0001.log',['audit_screen_assessment_unit.py',STAGE,'v2xt_0001']),
        ('additional_test_merge_audits.log',['merge_unit_assessment_audits.py',STAGE]),
        ('additional_test_analysis.log',['analyze_additional_test.py']),
        ('additional_test_final_report.log',['build_final_association_report.py'])]
    for name,args in steps:
        log=LOG/name;assert not log.exists(),name
        print('START FINAL STEP',args[0],flush=True)
        with log.open('w') as output:
            run=subprocess.run([sys.executable,str(OUT/args[0]),*args[1:]],cwd=ROOT,stdout=output,stderr=subprocess.STDOUT)
        print('END FINAL STEP',args[0],'return',run.returncode,flush=True)
        assert run.returncode==0,name
    print('ADDITIONAL TEST AUDIT AND REPORT COMPLETE',flush=True)


if __name__=='__main__':main()
