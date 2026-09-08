"""Register four Gaussian arms and the native scalar component control."""
from pathlib import Path

OUT = Path(__file__).resolve().parent
PREVIOUS = OUT.parent / 'icra_asymmetric_evidence'
NEW = ['marked_gaussian_evidence', 'marked_gaussian_evidence_no_curvature',
       'marked_gaussian_evidence_no_history', 'marked_gaussian_evidence_no_mark']
OLD = ['marked_asymmetric']
REF = ['marked_lineage', 'marked_er', 'marked_conservative', 'marked_ceiling_calibrated', 'marked_ceiling_score']


def replace(source, old, new, count=1):
    assert source.count(old) == count, (old, source.count(old), count)
    return source.replace(old, new)


def main():
    s = (PREVIOUS / 'run_sequences.py').read_text()
    s = s.replace('ICRA_ASYMMETRIC_EVIDENCE', 'ICRA_GAUSSIAN_EVIDENCE').replace('icra_asymmetric_evidence', 'icra_gaussian_evidence')
    s = s.replace('runAsymmetricReplay', 'runGaussianEvidenceReplay').replace('ASYMMETRIC', 'GAUSSIAN EVIDENCE').replace('asymmetric stage', 'Gaussian evidence stage')
    (OUT / 'run_sequences.py').write_text(s)
    s = (PREVIOUS / 'register_round.py').read_text()
    s = s.replace('analyze_asymmetric.py', 'analyze_gaussian.py').replace('exact_si_node_frames', 'exact_as_node_frames')
    s = s.replace('asymmetric-current-evidence-v1', 'coherent-gaussian-evidence-v1')
    s = s.replace("primary='marked_asymmetric'", "primary='marked_gaussian_evidence'")
    begin, end = s.index('                  arms='), s.index('                  primary_references=')
    s = s[:begin] + f"                  arms={NEW!r},\n                  arms_by_cohort={{'development': {NEW!r}, 'seen_transfer': {NEW+OLD!r}}},\n                  references_by_cohort={{'development': {REF+OLD!r}, 'seen_transfer': {REF!r}}},\n" + s[end:]
    s = replace(s, "component_controls=['marked_selective', 'marked_selective_signed']", "component_controls=['marked_asymmetric']")
    s = replace(s, "'make_audit.py', 'probability_audit.inc', 'make_drivers.py']", "'make_audit.py', 'probability_audit.inc', 'gaussian_audit.py', 'make_drivers.py']")
    s = replace(s, "'icra_selective_innovation/summary_development.json', 'icra_selective_innovation/ROUND_FREEZE.json']]",
                "'icra_asymmetric_evidence/summary_development.json', 'icra_asymmetric_evidence/ROUND_FREEZE.json']]")
    s = s.replace('and the no-negative M-SI control', 'and the scalar M-AE control')
    s = s.replace('ASYMMETRIC ROUND FIXED: M-AE primary; three new arms; two declared previous component controls;',
                  'GAUSSIAN EVIDENCE ROUND FIXED: M-GE primary; four new arms; native M-AE component control;')
    (OUT / 'register_round.py').write_text(s)
    print('Generated registered Gaussian evidence stages and native scalar control.')


if __name__ == '__main__':
    main()
