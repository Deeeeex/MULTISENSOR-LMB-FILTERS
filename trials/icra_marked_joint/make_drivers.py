"""Reuse the fixed stage scheduler and continuation check for the joint pair."""
from pathlib import Path

OUT = Path(__file__).resolve().parent
PRIOR = OUT.parent / 'icra_tempered_iteration'


def main():
    source = (PRIOR / 'run_sequences.py').read_text()
    source = source.replace('ICRA_TEMPERED_ITERATION', 'ICRA_MARKED_JOINT')
    source = source.replace('icra_tempered_iteration', 'icra_marked_joint').replace('runTemperedReplay', 'runMarkedJointReplay')
    source = source.replace('TEMPERED', 'MARKED JOINT').replace('tempered', 'marked joint').replace('Tempered', 'Marked joint')
    source = source.replace("r['files'] == 6", "r['files'] == 4").replace('6 * len(units)', '4 * len(units)')
    (OUT / 'run_sequences.py').write_text(source)
    source = (PRIOR / 'register_round.py').read_text()
    source = source.replace('analyze_tempered.py', 'analyze_marked_joint.py')
    source = source.replace("'evidence-tempered-recency-v1'", "'marked-current-innovation-v1'")
    source = source.replace("primary='marked_tempered_calibrated'", "primary='marked_joint_evidence'")
    old = "arms=['marked_tempered_calibrated', 'marked_tempered_score', 'marked_tempered_association']"
    assert source.count(old) == 1
    source = source.replace(old, "arms=['marked_joint_evidence', 'marked_joint_evidence_recency']")
    source = source.replace("'analyze_marked_joint.py', 'run_sequences.py', 'register_round.py']",
                            "'analyze_marked_joint.py', 'run_sequences.py', 'register_round.py',\n                                     'make_analysis.py', 'probability_audit.inc', 'make_drivers.py']")
    source = source.replace('TEMPERED ROUND FIXED: T-C primary; three arms;',
                            'MARKED JOINT ROUND FIXED: M-JE primary; two arms;')
    (OUT / 'register_round.py').write_text(source)
    print('Marked joint stage drivers generated; both arms and the original continuation criterion retained.')


if __name__ == '__main__':
    main()
