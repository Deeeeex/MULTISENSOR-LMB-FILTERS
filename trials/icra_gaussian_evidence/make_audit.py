"""Keep established scoring and extend it to independent Gaussian reconstruction."""
from pathlib import Path

OUT = Path(__file__).resolve().parent
PREVIOUS = OUT.parent / 'icra_asymmetric_evidence'


def replace(source, old, new, count=1):
    assert source.count(old) == count, (old, source.count(old), count)
    return source.replace(old, new)


def probability():
    s = (PREVIOUS / 'probability_audit.inc').read_text()
    s = s.replace('reshape(-1, 37)', 'reshape(-1, 60)')
    s = s.replace('marked_asymmetric_no_mark', 'marked_gaussian_evidence_no_mark')
    s = s.replace('marked_asymmetric_no_history', 'marked_gaussian_evidence_no_history')
    s = replace(s, '    if arm in OLD_ARMS:\n        expected_negative[:] = 0\n', '')
    s = replace(s, "    if arm == 'marked_selective_signed':\n        fresh = gates*delta\n", '')
    s = replace(s, '    for column, value in [(6, expected), (7, r0), (8, er), (9, expected)]:',
                "    columns = [(6, expected), (7, r0), (8, er), (9, expected)] if arm in OLD_ARMS else [(37, expected), (7, r0), (8, er)]\n    for column, value in columns:")
    s = replace(s, "    if arm == 'marked_selective':\n        assert np.all(expected >= np.minimum(r0, er)-2e-12)\n", '')
    start = s.index('    if arm == PRIMARY:')
    s = s[:start] + '''    diagnostic.update(check_gaussians(run, records, original, present, active, joint, beta, logits, delta, gates, negative_gates))
    if arm == PRIMARY:
        delivery, poses = np.asarray(data['delivered']), np.asarray(data['positions'])
        rules = ['scalar_AS', 'new_r_old_space', 'old_r_new_space', 'candidate']
        effects = {rule: {key: [] for key in METRICS[:6]} for rule in rules}
        alternatives = {}
        for rule in rules:
            alternative = records.copy()
            if rule in ['scalar_AS', 'new_r_old_space']:
                alternative[:, 4:6] = records[:, 38:40]
            if rule in ['scalar_AS', 'old_r_new_space']:
                alternative[:, 9] = records[:, 37]
            alternatives[rule] = alternative
        for t in range(len(data['time'])):
            for n in range(2):
                mask = (records[:, 0] == t+1) & (records[:, 1] == n+1)
                if not delivery[n, 1-n, t]:
                    assert not mask.any()
                    continue
                for rule in rules:
                    value = score(data['truth'][t], counterfactual(alternatives[rule][mask], 9, poses[:, :, t]))
                    if rule == 'candidate':
                        assert np.isclose(value['ospa'], run['ospa'][n][t], atol=1e-8, rtol=1e-9)
                    for key in effects[rule]:
                        effects[rule][key].append(value[key])
        diagnostic['same_input'] = {rule: {key: float(np.mean(v)) for key, v in values.items()}
                                    for rule, values in effects.items()}
    return diagnostic
'''
    return s


def main():
    audit = probability()
    (OUT / 'probability_audit.inc').write_text(audit)
    s = (PREVIOUS / 'analyze_asymmetric.py').read_text()
    s = replace(s, "PRIMARY = 'marked_asymmetric'", "PRIMARY = 'marked_gaussian_evidence'")
    s = replace(s, "NEW_ARMS = [PRIMARY, 'marked_asymmetric_no_history', 'marked_asymmetric_no_mark']",
                "NEW_ARMS = [PRIMARY, 'marked_gaussian_evidence_no_curvature', 'marked_gaussian_evidence_no_history', 'marked_gaussian_evidence_no_mark']")
    s = replace(s, "OLD_ARMS = ['marked_selective', 'marked_selective_signed']", "OLD_ARMS = ['marked_asymmetric']")
    s = replace(s, "ASYMMETRIC = OUT.parent / 'icra_selective_innovation'", "ASYMMETRIC = OUT.parent / 'icra_asymmetric_evidence'\nfrom gaussian_audit import check_gaussians")
    begin, end = s.index('def audit_probability('), s.index('\ndef audit_file(')
    s = s[:begin] + audit.rstrip() + '\n\n' + s[end:]
    s = s.replace('asymmetric-current-evidence-v1', 'coherent-gaussian-evidence-v1').replace('asymmetric-v1', 'gaussian-evidence-v1')
    s = replace(s, 'width = 232 if arm in NEW_ARMS else 224', 'width = 352 if arm in NEW_ARMS else 232')
    s = replace(s, "['marked_conservative', 'marked_ceiling_score', 'marked_selective']]",
                "['marked_conservative', 'marked_ceiling_score', 'marked_asymmetric']]")
    s = s.replace('Asymmetric arms 232 B/object; selective controls 224 B/object.',
                  'Gaussian evidence arms 352 B/object; scalar M-AE control 232 B/object. Different summaries and costs, not an equal-byte comparison.')
    begin, end = s.index('def preflight('), s.index('\n\ndef main():')
    pre = s[begin:end].replace('marked_selective', 'marked_asymmetric').replace('SI port parity', 'AS port parity')
    pre = pre.replace('exact_si_', 'exact_as_').replace('si_trajectory_', 'as_trajectory_')
    pre = replace(pre, "['runtimeSeconds', 'iterationRecords', 'localIncrementRecords']", "['runtimeSeconds', 'iterationRecords', 'localGaussianRecords', 'packetGaussianRecords']")
    pre = pre.replace('a.shape == (len(b), 37) and b.shape[1] == 35', 'a.shape == (len(b), 60) and b.shape[1] == 37')
    pre = pre.replace('a[:, :35]', 'a[:, :37]').replace('a[:, 35:]', 'a[:, 37:]')
    pre = replace(pre, '                assert np.array_equal(local_a[:, :10], local_b)', '                assert np.array_equal(local_a, local_b)')
    pre = pre.replace('2352', '2940').replace('exact SI', 'exact AS')
    pre = replace(pre, 'scalar_radio_packet_and_extraction_checked=True,', 'scalar_radio_packet_and_extraction_checked=True,\n                  local_prior_posterior_ratio_full_gaussian_and_normalizer_checked=True,')
    s = s[:begin] + pre + s[end:]
    s = s.replace('ALL ASYMMETRIC', 'ALL GAUSSIAN EVIDENCE').replace('ASYMMETRIC AUDITED', 'GAUSSIAN EVIDENCE AUDITED')
    s = s.replace('ASYMMETRIC PREFLIGHT', 'GAUSSIAN EVIDENCE PREFLIGHT').replace('Selective unit is not complete', 'Gaussian evidence unit is not complete')
    (OUT / 'analyze_gaussian.py').write_text(s)
    print('Generated complete Gaussian and scalar audit with native M-AE parity.')


if __name__ == '__main__':
    main()
