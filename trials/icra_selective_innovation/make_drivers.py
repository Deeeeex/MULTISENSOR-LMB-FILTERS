"""Adapt the existing fixed cohort scheduler, with the stronger advance gate."""
from pathlib import Path

OUT = Path(__file__).resolve().parent
PRIOR = OUT.parent / 'icra_tempered_iteration'


def main():
    source = (PRIOR / 'run_sequences.py').read_text()
    source = source.replace('ICRA_TEMPERED_ITERATION', 'ICRA_SELECTIVE_INNOVATION')
    source = source.replace('icra_tempered_iteration', 'icra_selective_innovation').replace('runTemperedReplay', 'runSelectiveReplay')
    source = source.replace('TEMPERED', 'SELECTIVE').replace('tempered', 'selective').replace('Tempered', 'Selective')
    source = source.replace("r['files'] == 6", "r['files'] == 8").replace('6 * len(units)', '8 * len(units)')
    (OUT / 'run_sequences.py').write_text(source)
    source = (PRIOR / 'register_round.py').read_text()
    source = source.replace('analyze_tempered.py', 'analyze_selective.py').replace('exact_er_node_frames', 'exact_cr_node_frames')
    source = source.replace('evidence-tempered-recency-v1', 'selective-current-innovation-v1')
    source = source.replace("primary='marked_tempered_calibrated'", "primary='marked_selective'")
    old = "arms=['marked_tempered_calibrated', 'marked_tempered_score', 'marked_tempered_association']"
    assert source.count(old) == 1
    source = source.replace(old, "arms=['marked_selective', 'marked_selective_no_history', 'marked_selective_no_mark', 'marked_selective_signed']")
    source = source.replace("'analyze_selective.py', 'run_sequences.py', 'register_round.py']",
                            "'analyze_selective.py', 'run_sequences.py', 'register_round.py',\n                                     'make_analysis.py', 'probability_audit.inc', 'make_drivers.py']")
    source = source.replace('no higher than M-CR in both conditions.', 'no higher than M-CR and M-ECR-S in both conditions.')
    source = source.replace('TEMPERED ROUND FIXED: T-C primary; three arms;', 'SELECTIVE ROUND FIXED: M-SI primary; four arms;')
    (OUT / 'register_round.py').write_text(source)
    print('Generated selective stage scheduler and stronger continuation rule.')


if __name__ == '__main__':
    main()
