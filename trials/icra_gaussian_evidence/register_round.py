"""Fix the primary and complete development stages after implementation parity."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    target = OUT / 'ROUND_FREEZE.json'
    assert not target.exists(), 'Never replace an executed round registration.'
    preflight = json.loads((OUT / 'preflight_audit.json').read_text())
    assert preflight['passed'] and preflight['exact_as_node_frames'] == 588
    assert preflight['auditor_sha256'] == sha(OUT / 'analyze_gaussian.py')
    for name, expected in preflight['input_sha256'].items():
        assert sha(ROOT / name) == expected, name
    source = json.loads((OUT / 'source_sha256.json').read_text())
    for name, expected in source.items():
        assert sha(ROOT / name) == expected, name
    old = json.loads((OUT.parent / 'icra_fusion_holdout/METHOD_FREEZE.json').read_text())
    files = [OUT / name for name in ['PROTOCOL.md', 'source_sha256.json', 'preflight_audit.json',
                                     'analyze_gaussian.py', 'run_sequences.py', 'register_round.py',
                                     'make_audit.py', 'probability_audit.inc', 'gaussian_audit.py', 'make_drivers.py']]
    files += [OUT.parent / relative for relative in [
        'icra_fusion_holdout/analyze_holdout.py', 'icra_fusion_holdout/input_manifest.json',
        'icra_fusion_holdout/input_audit.json', 'icra_marked_control/analyze_control.py',
        'icra_marked_control/analyze_full.py', 'icra_marked_control/summary_control.json',
        'icra_method_iteration/analyze_development.py', 'icra_external_fusion/analyze_v2v4real.py',
        'icra_external_fusion/analyze_case_studies.py',
        'icra_asymmetric_evidence/summary_development.json', 'icra_asymmetric_evidence/ROUND_FREEZE.json']]
    result = dict(protocol='coherent-gaussian-evidence-v1', timestamp_utc=datetime.now(timezone.utc).isoformat(),
                  primary='marked_gaussian_evidence',
                  arms=['marked_gaussian_evidence', 'marked_gaussian_evidence_no_curvature', 'marked_gaussian_evidence_no_history', 'marked_gaussian_evidence_no_mark'],
                  arms_by_cohort={'development': ['marked_gaussian_evidence', 'marked_gaussian_evidence_no_curvature', 'marked_gaussian_evidence_no_history', 'marked_gaussian_evidence_no_mark'], 'seen_transfer': ['marked_gaussian_evidence', 'marked_gaussian_evidence_no_curvature', 'marked_gaussian_evidence_no_history', 'marked_gaussian_evidence_no_mark', 'marked_asymmetric']},
                  references_by_cohort={'development': ['marked_lineage', 'marked_er', 'marked_conservative', 'marked_ceiling_calibrated', 'marked_ceiling_score', 'marked_asymmetric'], 'seen_transfer': ['marked_lineage', 'marked_er', 'marked_conservative', 'marked_ceiling_calibrated', 'marked_ceiling_score']},
                  primary_references=['marked_lineage', 'marked_er'],
                  additional_references=['marked_conservative', 'marked_ceiling_calibrated', 'marked_ceiling_score'],
                  component_controls=['marked_asymmetric'],
                  conditions=old['conditions'], bootstrap=old['bootstrap'],
                  all_previous_real_data_outcomes_seen=True,
                  primary_selected_before_preflight=True, secondary_cannot_replace_primary=True,
                  continuation_rule='All nine development sequences complete; primary sequence-macro OSPA lower than M-No-age and M-ER in both conditions and no higher than M-CR, M-ECR-S and the scalar M-AE control in both conditions. Otherwise reject expansion, retain all secondary results.',
                  units=[dict(cohort='development', sequence=s, index=s) for s in range(9)] +
                        [dict(cohort='seen_transfer', sequence=s, index=i) for i, s in enumerate(old['units'])],
                  source_and_evidence_sha256={str(p.relative_to(ROOT)): sha(p) for p in files})
    for cohort in ['development', 'seen_transfer']:
        assert not list((OUT / f'results_{cohort}').glob('*.json.gz'))
    target.write_text(json.dumps(result, indent=2) + '\n')
    print('GAUSSIAN EVIDENCE ROUND FIXED: M-GE primary; four new arms; native M-AE component control; all nine development units first; no independent-holdout claim.', flush=True)


if __name__ == '__main__':
    main()
