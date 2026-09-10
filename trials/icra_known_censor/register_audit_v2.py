"""Preserve the failed audit and register a scalar-storage-only audit revision."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    destination=OUT/'AUDIT_FREEZE_V2.json';assert not destination.exists()
    assert not (OUT/'audit_known_censor_controls.json').exists() and not (OUT/'results/known_censor_refined').exists()
    freeze=json.loads((OUT/'FREEZE.json').read_text());cfg=json.loads((OUT/'stages/known_censor_controls.json').read_text())
    for name,h in {**freeze['configurations'],**cfg['source_sha256']}.items():assert sha(ROOT/name)==h,name
    receipts=[]
    def port(source_name,target_name,changes):
        source=OUT/source_name;target=OUT/target_name;assert not target.exists();content=source.read_text()
        for before,after in changes:assert content.count(before)==1,before;content=content.replace(before,after)
        target.write_text(content)
        receipts.append(dict(source=str(source.relative_to(ROOT)),source_sha256=sha(source),target=str(target.relative_to(ROOT)),
            target_sha256=sha(target),changes=[dict(before=a,after=b) for a,b in changes]))
    v2_check="    v2=json.loads((OUT/'AUDIT_FREEZE_V2.json').read_text())\n    for name,h in v2['source_sha256'].items():assert sha(ROOT/name)==h,name\n"
    old="    for name,h in cfg['source_sha256'].items():assert sha(ROOT/name)==h,name\n"
    native_function="\ndef storage_value(value):\n    if isinstance(value,np.generic):return value.item()\n    raise TypeError(type(value).__name__)\n"
    port('audit_stage.py','audit_stage_v2.py',[
        ("SUFFIX='_known_censor'\n","SUFFIX='_known_censor'\n"+native_function),
        ("destination=OUT/('audit_'+args.stage+'.json')","destination=OUT/('audit_v2_'+args.stage+'.json')"),
        (old,old+v2_check),
        ("_reimport.csv.gz'","_reimport_v2.csv.gz'"),
        ("OUT/'audit_known_censor_controls.json'","OUT/'audit_v2_known_censor_controls.json'"),
        ("[('scores_',rows),('target_frames_',timeline),('population_frames_',population_rows)]",
         "[('scores_v2_',rows),('target_frames_v2_',timeline),('population_frames_v2_',population_rows)]"),
        ("indent=2,allow_nan=False)+'\\n')","indent=2,allow_nan=False,default=storage_value)+'\\n')"),
    ])
    port('run_stage.py','run_stage_v2.py',[(old,old+v2_check),
        ("OUT/'audit_known_censor_controls.json'","OUT/'audit_v2_known_censor_controls.json'")])
    before="    freeze=json.loads((OUT/'FREEZE.json').read_text());assert freeze['passed'];protected={};scores={};diags={};paths={};target_count=0;event_count=0\n"
    note="        '控制审计 v1 在六条轨迹的全部数值与逐字段检查通过后，因 NumPy int64 无法直接写入 JSON 而退出。v2 只将 NumPy 标量转换为等值 Python 标量并使用新审计输出路径；原源码、部分 CSV/事件文件及失败日志保留。原生算法、六条控制、六条待运行干预配置、公式、容差与门槛未改变。v2 完成了整个审计。','',\n"
    port('finish.py','finish_v2.py',[(before,before+v2_check),
        ("auditpath=OUT/('audit_'+stage+'.json')","auditpath=OUT/('audit_v2_'+stage+'.json')"),
        ("OUT/('target_frames_'+stage+'.csv')","OUT/('target_frames_v2_'+stage+'.csv')"),
        ("OUT/('population_frames_'+stage+'.csv')","OUT/('population_frames_v2_'+stage+'.csv')"),
        ("        '公平比较：']","        '公平比较：']"),
        ("    lines=['# 使用本地已知存在率：完整递归因果检验','',\n","    lines=['# 使用本地已知存在率：完整递归因果检验','',\n"+note),
    ])
    correction=dict(passed=True,created_utc=datetime.now(timezone.utc).isoformat(),reason='NumPy int64 in final JSON serialization after six complete control checks',
        native_or_formula_changes=False,refined_outcomes_exposed=False,ports=receipts)
    patchpath=OUT/'AUDIT_STORAGE_FIX.json';patchpath.write_text(json.dumps(correction,indent=2)+'\n')
    sources={}
    protected=[OUT/'FREEZE.json',OUT/'stages/known_censor_controls.json',OUT/'stages/known_censor_refined.json',
        OUT/'runtime_known_censor_controls.json',ROOT/'RUN/ICRA_KNOWN_CENSOR/audit_controls_v1_failure.log',patchpath,Path(__file__)]
    protected+=[ROOT/r[k] for r in receipts for k in ['source','target']]
    protected+=list(OUT.glob('*known_censor_controls.csv'))+list((OUT/'results/known_censor_controls').glob('*.gz'))
    for path in protected:sources[str(path.relative_to(ROOT))]=sha(path)
    destination.write_text(json.dumps(dict(passed=True,created_utc=datetime.now(timezone.utc).isoformat(),source_sha256=sources,
        preserved_native_controls=6,unrun_refined_trajectories=6,scope='Storage and output-path revision only; original native freeze remains authoritative'),indent=2)+'\n')
    print('AUDIT V2 STORAGE REVISION FROZEN',len(sources),'files; native rules unchanged',flush=True)

if __name__=='__main__':main()
