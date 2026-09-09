"""Bind the declared two-factor diagnostic before any alternate output scores."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    destination = OUT / 'SCREEN_FREEZE.json'
    assert not destination.exists() and not (OUT / 'SCREEN_RESULTS.json').exists()
    configurations = [OUT.parent / 'icra_miss_history/stages' / f'miss_history_{s}.json'
                      for s in ['preflight', 'screen']]
    prior = [json.loads(p.read_text()) for p in configurations]
    units = [u for cfg in prior for u in cfg['units'] if u['dataset'] != 'v2x_test_mechanism']
    assert len(units) == 14 and len({u['sequence'] for u in units}) == 14
    sources = {k: v for cfg in prior for k, v in cfg['source_sha256'].items()}
    audit_paths = [OUT.parent / p for p in [
        'icra_temporal_association/audit_association_instrumentation_development.json',
        'icra_reviewer_revision/audit_controls_development.json',
        'icra_projected_admission/audit_final_v2x_evaluation.json']]
    audited = {}
    for path in audit_paths:
        audit = json.loads(path.read_text()); assert audit['passed']
        audited.update(audit['inputs'])
    cells = []
    for unit in units:
        vv = unit['dataset'] == 'v2v_development'
        gs_directory = ('icra_reviewer_revision/results/controls_development' if vv else
                        'icra_projected_admission/results/final_v2x_evaluation')
        for condition in ['reliable', 'intermittent']:
            paths = {'GCE': ROOT / unit['reference_paths'][condition],
                     'Guarded Scalar': OUT.parent / gs_directory /
                     f"{unit['sequence']}_{condition}_marked_gaussian_evidence_guarded_scalar.json.gz"}
            for backend, path in paths.items():
                key = str(path.relative_to(ROOT)); digest = sha(path)
                assert audited[key] == digest
                if backend == 'GCE': assert unit['reference_sha256'][condition] == digest
                cells.append(dict(dataset=unit['dataset'], sequence=unit['sequence'],
                    condition=condition, backend=backend, path=key, sha256=digest,
                    recording=unit.get('source_recording', unit['recording']), data_path=unit['data_path'],
                    ratios_path=unit.get('ratios_path', unit['data_path'])))
                sources[key] = digest
        for field in ['data_path', 'marks_path', 'ratios_path', 'pose_path']:
            if field in unit: sources[unit[field]] = sha(ROOT / unit[field])
    protected = configurations + audit_paths + list(OUT.glob('*.py')) + [OUT / 'PROTOCOL.md']
    protected += [OUT.parent / p for p in [
        'icra_reviewer_revision/review_gaussian_audit.py',
        'icra_method_iteration/analyze_development.py',
        'icra_external_fusion/analyze_case_studies.py',
        'icra_reunion_fusion/analyze_results.py',
        'icra_reunion_fusion/analyze_validation.py',
        'icra_gaussian_evidence/RESULTS_development_CN.md',
        'icra_range_detection/ASSIGNMENT_MASS_DIAGNOSTIC.json']]
    sources.update({str(p.relative_to(ROOT)): sha(p) for p in protected})
    for key, digest in sources.items(): assert sha(ROOT / key) == digest, key
    cfg = dict(protocol='icra-joint-admission-fixed-input-v1',
        created_utc=datetime.now(timezone.utc).isoformat(),
        rules=['original', 'joint_mark', 'conditional', 'joint'], primary='joint',
        selection_backend='GCE', references=['original', 'conditional'],
        conditions=['reliable', 'intermittent'], cells=cells, source_sha256=sources,
        exposure='All inputs previously exposed; one-step substitution, no recursive result')
    destination.write_text(json.dumps(cfg, indent=2, allow_nan=False) + '\n')
    print('FROZEN', len(cells), 'source runs;', len(sources), 'protected files', flush=True)


if __name__ == '__main__': main()
