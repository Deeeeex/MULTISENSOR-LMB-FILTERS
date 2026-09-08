"""Reuse full scoring with independent branch-support and source-link checks."""
from pathlib import Path

OUT = Path(__file__).resolve().parent
PREVIOUS = OUT.parent / 'icra_selective_innovation'


def replace(source, old, new, count=1):
    assert source.count(old) == count, (old, source.count(old), count)
    return source.replace(old, new)


def probability():
    s = (PREVIOUS / 'probability_audit.inc').read_text()
    s = s.replace("ARMS + ['marked_conservative']", "ARMS + OLD_ARMS")
    s = s.replace('reshape(-1, 10)', 'reshape(-1, 12)').replace('reshape(-1, 35)', 'reshape(-1, 37)')
    s = s.replace('marked_selective_no_mark', 'marked_asymmetric_no_mark')
    s = replace(s, "    records = np.asarray(run['iterationRecords']", """    pd, negative_local = local[:, 10], local[:, 11]
    assert np.isin(pd, [0, .9]).all() and np.array_equal(pd > 0, local[:, 7].astype(bool))
    expected_negative = (1-local[:, 9])*pd/(2-pd)
    if arm in OLD_ARMS:
        expected_negative[:] = 0
    assert np.allclose(negative_local, expected_negative, atol=1e-14, rtol=0)
    records = np.asarray(run['iterationRecords']""")
    s = replace(s, '    gates, beta = records[:, 26:28], records[:, 28:30]',
                '    gates, beta = records[:, 26:28], records[:, 28:30]\n    negative_gates = records[:, 35:37]')
    begin = s.index("    if arm == 'marked_conservative':")
    body_begin = s.index('    else:\n', begin)+len('    else:\n')
    end = s.index('    expected, r0, er', body_begin)
    body = '\n'.join(line[4:] if line.startswith('    ') else line for line in s[body_begin:end].split('\n'))
    s = s[:begin]+body+s[end:]
    s = replace(s, '    assert np.all((gates >= 0) & (gates <= 1))',
                '    assert np.all((gates >= 0) & (gates <= 1))\n    assert np.all((negative_gates >= 0) & (negative_gates <= 1))\n    assert np.all(negative_gates[stamps != records[:, 0, None]] == 0)')
    s = replace(s, "        assert gates[i, side] == expected_gate, (arm, key, 'received gate')",
                "        assert gates[i, side] == expected_gate, (arm, key, 'received gate')\n        expected_negative_gate = row[11] if stamps[i, side] == records[i, 0] else 0\n        assert negative_gates[i, side] == expected_negative_gate, (arm, key, 'received negative gate')")
    s = s.replace('marked_selective_no_history', 'marked_asymmetric_no_history')
    s = replace(s, "    fresh = delta if arm == 'marked_selective_signed' else np.maximum(delta, 0)\n    boost = joint*((active-beta)*gates*fresh).sum(1)",
                "    fresh = gates*np.maximum(delta, 0)+negative_gates*np.minimum(delta, 0)\n    if arm == 'marked_selective_signed':\n        fresh = gates*delta\n    boost = joint*((active-beta)*fresh).sum(1)")
    s = replace(s, "    if arm in [PRIMARY, 'marked_asymmetric_no_mark']:\n        assert np.all(expected >= np.minimum(r0, er)-2e-12)",
                "    if arm == 'marked_selective':\n        assert np.all(expected >= np.minimum(r0, er)-2e-12)")
    s = replace(s, 'mean_gate=float(gates[active].mean()), mean_boost=float(boost.mean()))',
                'mean_gate=float(gates[active].mean()), mean_negative_gate=float(negative_gates[active].mean()),\n                      mean_boost=float(boost.mean()))')
    s = s.replace("['no_age', 'ER', 'CR', 'candidate']", "['no_age', 'ER', 'CR', 'no_negative', 'candidate']")
    s = replace(s, '        conservative_records[:, 9] = np.minimum(r0, er)',
                '        conservative_records[:, 9] = np.minimum(r0, er)\n        positive_records = records.copy()\n        positive_records[:, 9] = expit((beta*logits).sum(1)+records[:, 10]+joint*((active-beta)*gates*np.maximum(delta, 0)).sum(1))')
    s = s.replace("('CR', 9), ('candidate', 9)", "('CR', 9), ('no_negative', 9), ('candidate', 9)")
    s = replace(s, "                    selected = conservative_records[mask] if rule == 'CR' else subset",
                "                    selected = conservative_records[mask] if rule == 'CR' else subset\n                    if rule == 'no_negative':\n                        selected = positive_records[mask]")
    return s


def main():
    audit = probability()
    (OUT / 'probability_audit.inc').write_text(audit)
    s = (PREVIOUS / 'analyze_selective.py').read_text()
    s = replace(s, "PRIMARY = 'marked_selective'", "PRIMARY = 'marked_asymmetric'")
    s = replace(s, "ARMS = [PRIMARY, 'marked_selective_no_history', 'marked_selective_no_mark', 'marked_selective_signed']",
                "NEW_ARMS = [PRIMARY, 'marked_asymmetric_no_history', 'marked_asymmetric_no_mark']\nOLD_ARMS = ['marked_selective', 'marked_selective_signed']\nARMS = NEW_ARMS.copy()")
    s = replace(s, "REFERENCES = ['marked_lineage'", "BASE_REFERENCES = ['marked_lineage'")
    s = replace(s, "              'marked_ceiling_calibrated', 'marked_ceiling_score']",
                "              'marked_ceiling_calibrated', 'marked_ceiling_score']\nREFERENCES = BASE_REFERENCES + OLD_ARMS\nSELECTIVE = OUT.parent / 'icra_selective_innovation'")
    begin, end = s.index('def audit_probability('), s.index('\n\ndef audit_file(')
    s = s[:begin]+audit.rstrip()+s[end:]
    s = s.replace('selective-current-innovation-v1', 'asymmetric-current-evidence-v1').replace('selective-v1', 'asymmetric-v1')
    s = replace(s, "width = 208 if arm == 'marked_conservative' else 224", "width = 232 if arm in NEW_ARMS else 224")
    s = replace(s, "    if arm != 'marked_conservative':", """    if arm in OLD_ARMS:
        assert cohort == 'development'
        summary = json.loads((SELECTIVE / 'summary_development.json').read_text())
        path = SELECTIVE / 'results_development' / f'{name}_{condition}_{arm}.json.gz'
        assert sha(path) == summary['inputs'][str(path.relative_to(ROOT))]
        data = read(path)
        row, matched = score_run(data, data['runs'])
        expected = next(r for r in summary['runs'] if r['sequence'] == name and r['condition'] == condition and r['arm'] == arm)
        for key in METRICS:
            assert np.isclose(row[key], expected[key], atol=1e-10, rtol=0)
        row.update(sequence=name, condition=condition, arm=arm, frames=len(data['time']))
        return data, row, matched, path
    if arm != 'marked_conservative':""")
    s = replace(s, "['marked_conservative', 'marked_ceiling_score']]",
                "['marked_conservative', 'marked_ceiling_score', 'marked_selective']]")
    s = s.replace('Selective arms 224 B/object.', 'Asymmetric arms 232 B/object; selective controls 224 B/object.')
    begin, end = s.index('def preflight('), s.index('\n\ndef main():')
    section = s[begin:end]
    section = section.replace('marked_conservative', 'marked_selective')
    section = replace(section, "CONTROL / 'results_development'", "SELECTIVE / 'results_development'")
    section = section.replace('CR port parity', 'SI port parity').replace('exact_cr_', 'exact_si_').replace('cr_trajectory_', 'si_trajectory_')
    section = section.replace("a.shape == (len(b), 35) and b.shape[1] == 26", "a.shape == (len(b), 37) and b.shape[1] == 35")
    section = section.replace('a[:, :26]', 'a[:, :35]').replace('a[:, 26:]', 'a[:, 35:]')
    section = replace(section, '                assert np.all(a[:, 35:] == 0)',
                      "                assert np.all(a[:, 35:] == 0)\n                local_a = np.asarray(data['runs']['localIncrementRecords'], float)\n                local_b = np.asarray(original['runs']['localIncrementRecords'], float)\n                assert np.array_equal(local_a[:, :10], local_b)")
    section = section.replace('2940', '2352').replace('exact CR', 'exact SI')
    s = s[:begin]+section+s[end:]
    s = replace(s, "entry['files'] == 8", "entry['files'] == 2*len(ARMS)")
    s = replace(s, 'def main():\n', 'def main():\n    global ARMS, REFERENCES\n')
    s = replace(s, '    source = source_check()',
                "    ARMS = NEW_ARMS + (OLD_ARMS if args.cohort == 'seen_transfer' else [])\n    REFERENCES = BASE_REFERENCES + (OLD_ARMS if args.cohort == 'development' else [])\n    source = source_check()")
    s = replace(s, "    frozen = json.loads((OUT / 'ROUND_FREEZE.json').read_text())",
                "    frozen = json.loads((OUT / 'ROUND_FREEZE.json').read_text())\n    assert ARMS == frozen['arms_by_cohort'][args.cohort]\n    assert REFERENCES == frozen['references_by_cohort'][args.cohort]")
    s = s.replace('SELECTIVE ', 'ASYMMETRIC ').replace('selective scalar', 'asymmetric scalar')
    (OUT / 'analyze_asymmetric.py').write_text(s)
    print('Generated complete asymmetric audit and exact saved-SI control parity.')


if __name__ == '__main__':
    main()
