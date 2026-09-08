"""Reuse the complete-stage scorer and replace rule-specific checks only."""
from pathlib import Path

OUT = Path(__file__).resolve().parent


def replace(source, old, new, count=1):
    assert source.count(old) == count, (old, source.count(old), count)
    return source.replace(old, new)


def main():
    source = (OUT.parent / 'icra_tempered_iteration/analyze_tempered.py').read_text()
    source = replace(source, "PRIMARY = 'marked_tempered_calibrated'", "PRIMARY = 'marked_selective'")
    source = replace(source, "ARMS = [PRIMARY, 'marked_tempered_score', 'marked_tempered_association']",
                     "ARMS = [PRIMARY, 'marked_selective_no_history', 'marked_selective_no_mark', 'marked_selective_signed']")
    source = source.replace('evidence-tempered-recency-v1', 'selective-current-innovation-v1')
    source = source.replace('tempered-v1', 'selective-v1')
    begin, end = source.index('def audit_probability('), source.index('\n\ndef audit_file(')
    source = source[:begin] + (OUT / 'probability_audit.inc').read_text().rstrip() + source[end:]
    source = replace(source, "width = 208 if arm == 'marked_er' else 216", "width = 208 if arm == 'marked_conservative' else 224")
    source = replace(source, "conservative = [r for r in primary if r['reference'] == 'marked_conservative']",
                     "conservative = [r for r in primary if r['reference'] in ['marked_conservative', 'marked_ceiling_score']]")
    source = source.replace('216 B/object', '224 B/object')
    begin, end = source.index('def preflight('), source.index('\n\ndef main():')
    section = source[begin:end]
    section = section.replace("['marked_er'] + ARMS", "['marked_conservative'] + ARMS")
    section = section.replace("arm == 'marked_er'", "arm == 'marked_conservative'")
    section = section.replace("r['arm'] == 'marked_er'", "r['arm'] == 'marked_conservative'")
    section = replace(section, "PORT / 'results_development_check' / f'0000_{condition}_marked_er.json.gz'",
                      "CONTROL / 'results_development' / f'0000_{condition}_marked_conservative.json.gz'")
    section = replace(section, "                    if key != 'runtimeSeconds':\n                        assert value == original['runs'][key], (condition, 'ER port parity', key)",
                      """                    if key not in ['runtimeSeconds', 'iterationRecords', 'localIncrementRecords']:
                        assert value == original['runs'][key], (condition, 'CR port parity', key)
                a = np.asarray(data['runs']['iterationRecords'], float)
                b = np.asarray(original['runs']['iterationRecords'], float)
                assert a.shape == (len(b), 35) and b.shape[1] == 26
                assert np.array_equal(a[:, :26], b, equal_nan=True)
                assert np.all(a[:, 26:] == 0)""")
    section = section.replace('exact_er_', 'exact_cr_').replace('full_er_fields_exact_except_runtime', 'cr_trajectory_metric_packet_and_original_diagnostics_exact')
    section = section.replace('2352', '2940').replace('exact ER', 'exact CR')
    source = source[:begin] + section + source[end:]
    source = replace(source, "entry['files'] == 6", "entry['files'] == 8")
    anchor = "    (OUT / f'summary_{cohort}.json').write_text"
    extra = """    result['preceding_marked_joint_comparison'] = []
    if cohort == 'development':
        previous = json.loads((OUT.parent / 'icra_marked_joint/summary_development.json').read_text())
        assert previous['sequences'] == 9 and not previous['continuation_gate_passed']
        result['preceding_marked_joint_aggregate'] = [r for r in previous['aggregate'] if r['arm'] in ['marked_joint_evidence', 'marked_joint_evidence_recency']]
        old = {(r['sequence'], r['condition'], r['arm']): r for r in previous['runs']}
        for condition in frozen['conditions']:
            for reference in ['marked_joint_evidence', 'marked_joint_evidence_recency']:
                a = [lookup[name, condition, PRIMARY] for name in names]
                b = [old[name, condition, reference] for name in names]
                result['preceding_marked_joint_comparison'].append(dict(condition=condition, candidate=PRIMARY, reference=reference,
                    **{key: interval([x[key]-y[key] for x,y in zip(a,b)], samples) for key in METRICS[:6]}))
    result['preceding_joint_note'] = 'Previous JE values come from the preserved independently rescored full development summary; they are not rerun or counted as newly rescored node-frames in this round.'
"""
    source = replace(source, anchor, extra + anchor)
    source = source.replace('tempered', 'selective').replace('TEMPERED', 'SELECTIVE').replace('Tempered', 'Selective')
    (OUT / 'analyze_selective.py').write_text(source)
    print('Generated independent selective audit, original CR parity and preceding JE comparison.')


if __name__ == '__main__':
    main()
