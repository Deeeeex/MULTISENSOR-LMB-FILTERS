"""Register distinct new-arm and component-control stages before full execution."""
from pathlib import Path

OUT = Path(__file__).resolve().parent
PRIOR = OUT.parent / 'icra_tempered_iteration'
NEW = ['marked_asymmetric', 'marked_asymmetric_no_history', 'marked_asymmetric_no_mark']
OLD = ['marked_selective', 'marked_selective_signed']
REF = ['marked_lineage', 'marked_er', 'marked_conservative', 'marked_ceiling_calibrated', 'marked_ceiling_score']


def replace(source, old, new, count=1):
    assert source.count(old) == count, (old, source.count(old), count)
    return source.replace(old, new)


def main():
    s = (PRIOR / 'run_sequences.py').read_text()
    s = s.replace('ICRA_TEMPERED_ITERATION', 'ICRA_ASYMMETRIC_EVIDENCE')
    s = s.replace('icra_tempered_iteration', 'icra_asymmetric_evidence').replace('runTemperedReplay', 'runAsymmetricReplay')
    s = s.replace('TEMPERED', 'ASYMMETRIC').replace('tempered', 'asymmetric').replace('Tempered', 'Asymmetric')
    s = replace(s, "    source = json.loads((OUT / 'source_sha256.json').read_text())",
                "    arms = frozen['arms_by_cohort'][args.cohort]\n    source = json.loads((OUT / 'source_sha256.json').read_text())")
    s = replace(s, "for arm in frozen['arms']]", 'for arm in arms]')
    s = s.replace("r['files'] == 6", "r['files'] == 2*len(arms)").replace('6 * len(units)', '2 * len(arms) * len(units)')
    (OUT / 'run_sequences.py').write_text(s)
    s = (PRIOR / 'register_round.py').read_text()
    s = s.replace('analyze_tempered.py', 'analyze_asymmetric.py').replace('exact_er_node_frames', 'exact_si_node_frames')
    s = s.replace('evidence-tempered-recency-v1', 'asymmetric-current-evidence-v1')
    s = s.replace("primary='marked_tempered_calibrated'", "primary='marked_asymmetric'")
    s = replace(s, "arms=['marked_tempered_calibrated', 'marked_tempered_score', 'marked_tempered_association'],",
                f"arms={NEW!r},\n                  arms_by_cohort={{'development': {NEW!r}, 'seen_transfer': {NEW+OLD!r}}},\n                  references_by_cohort={{'development': {REF+OLD!r}, 'seen_transfer': {REF!r}}},")
    s = replace(s, "additional_references=['marked_conservative', 'marked_ceiling_calibrated', 'marked_ceiling_score'],",
                "additional_references=['marked_conservative', 'marked_ceiling_calibrated', 'marked_ceiling_score'],\n                  component_controls=['marked_selective', 'marked_selective_signed'],")
    s = replace(s, "'analyze_asymmetric.py', 'run_sequences.py', 'register_round.py']",
                "'analyze_asymmetric.py', 'run_sequences.py', 'register_round.py',\n                                     'make_audit.py', 'probability_audit.inc', 'make_drivers.py']")
    s = replace(s, "        'icra_external_fusion/analyze_case_studies.py']]",
                "        'icra_external_fusion/analyze_case_studies.py',\n        'icra_selective_innovation/summary_development.json', 'icra_selective_innovation/ROUND_FREEZE.json']]")
    s = s.replace('no higher than M-CR in both conditions.', 'no higher than M-CR, M-ECR-S and the no-negative M-SI control in both conditions.')
    s = s.replace('TEMPERED ROUND FIXED: T-C primary; three arms;', 'ASYMMETRIC ROUND FIXED: M-AE primary; three new arms; two declared previous component controls;')
    (OUT / 'register_round.py').write_text(s)
    print('Generated asymmetric scheduler with complete development and declared transfer component controls.')


if __name__ == '__main__':
    main()
