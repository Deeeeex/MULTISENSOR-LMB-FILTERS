"""Add unit scheduling around the unchanged complete assessment audit body."""
from pathlib import Path
import hashlib
import json

OUT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    source=OUT/'audit_screen_assessment_v2.py'
    target=OUT/'audit_screen_assessment_unit.py'
    assert not target.exists()
    text=source.read_text()
    changes=[
        ('import csv\n','import csv\nimport hashlib\n'),
        ("    parser.add_argument('stage')\n", "    parser.add_argument('stage')\n    parser.add_argument('unit')\n"),
        ("    cfg, runtime = json.loads(cfgpath.read_text()), json.loads(runtimepath.read_text())\n    assert len(runtime) == len(cfg['units'])\n",
         "    cfg = json.loads(cfgpath.read_text())\n    runtime_text = runtimepath.read_text()\n    runtime_snapshot = json.loads(runtime_text)\n    cfg = dict(cfg, units=[u for u in cfg['units'] if u['sequence'] == args.unit])\n    runtime = [r for r in runtime_snapshot if r['sequence'] == args.unit]\n    assert len(runtime) == len(cfg['units']) == 1\n"),
        ("        config_sha256=sha(cfgpath), runtime_sha256=sha(runtimepath),\n",
         "        config_sha256=sha(cfgpath), runtime_sha256=hashlib.sha256(runtime_text.encode()).hexdigest(),\n        execution_unit=args.unit, native_unit=runtime[0], runtime_snapshot=runtime_snapshot, partial_unit_audit=True,\n"),
        ("    destination = OUT / ('audit_' + args.stage + '.json')\n",
         "    folder = OUT / 'partial_audits' / args.stage\n    folder.mkdir(parents=True, exist_ok=True)\n    destination = folder / (args.unit + '.json')\n"),
        ("    with (OUT / ('scores_' + args.stage + '.csv')).open('w') as handle:\n",
         "    with (folder / (args.unit + '.csv')).open('w') as handle:\n"),
        ("    print('ASSOCIATION STAGE AUDIT PASSED', args.stage, result['audited_robot_frames'], 'robot-frames', flush=True)\n",
         "    print('ASSOCIATION UNIT AUDIT PASSED', args.stage, args.unit, result['audited_robot_frames'], 'robot-frames', flush=True)\n")]
    for before,after in changes:
        assert text.count(before)==1,before
        text=text.replace(before,after)
    target.write_text(text)
    receipt=dict(source_sha256=sha(source),target_sha256=sha(target),generator_sha256=sha(Path(__file__)),
        exact_replacements=[dict(before=a,after=b) for a,b in changes],
        purpose='Scope the same audit loop to one completed native unit. Preserve all physical, packet, association, history, density, domain and score checks. Record the exact runtime snapshot; final merge requires all native units completed.')
    (OUT/'UNIT_AUDIT_SCHEDULING_PATCH.json').write_text(json.dumps(receipt,indent=2)+'\n')


if __name__=='__main__':main()
