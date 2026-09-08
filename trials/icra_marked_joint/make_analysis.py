"""Adapt the previously verified full-stage audit; replace only rule-specific checks."""
from pathlib import Path

OUT = Path(__file__).resolve().parent


def replace(source, old, new, count=1):
    assert source.count(old) == count, (old, source.count(old), count)
    return source.replace(old, new)


def main():
    source = (OUT.parent / 'icra_tempered_iteration/analyze_tempered.py').read_text()
    source = replace(source, "PRIMARY = 'marked_tempered_calibrated'", "PRIMARY = 'marked_joint_evidence'")
    source = replace(source, "ARMS = [PRIMARY, 'marked_tempered_score', 'marked_tempered_association']",
                     "ARMS = [PRIMARY, 'marked_joint_evidence_recency']")
    source = replace(source, "'evidence-tempered-recency-v1'", "'marked-current-innovation-v1'")
    source = replace(source, "'tempered-v1'", "'marked-joint-v1'")
    begin, end = source.index('def audit_probability('), source.index('\n\ndef audit_file(')
    replacement = (OUT / 'probability_audit.inc').read_text()
    source = source[:begin] + replacement.rstrip() + source[end:]
    source = replace(source, "                    if key != 'runtimeSeconds':\n                        assert value == original['runs'][key], (condition, 'ER port parity', key)",
                     "                    if key not in ['runtimeSeconds', 'iterationRecords', 'localIncrementRecords']:\n                        assert value == original['runs'][key], (condition, 'ER port parity', key)\n                a = np.asarray(data['runs']['iterationRecords'], float)\n                b = np.asarray(original['runs']['iterationRecords'], float)\n                assert a.shape == b.shape and np.allclose(a[:, :19], b[:, :19], atol=2e-12, rtol=0, equal_nan=True)\n                assert np.allclose(a[:, 21:24], b[:, 21:24], atol=1e-10, rtol=0)")
    source = source.replace("full_er_fields_exact_except_runtime=True", "er_trajectory_metric_and_packet_fields_exact=True, er_common_scalar_diagnostics_match=True")
    source = source.replace('2352', '1764')
    source = replace(source, "entry['files'] == 6", "entry['files'] == 4")
    source = source.replace('tempered', 'marked joint').replace('TEMPERED', 'MARKED JOINT').replace('Tempered', 'Marked joint')
    # Integrate the complete earlier unmarked JE controls without tracking again.
    source = replace(source, 'rows, common, provenance, diagnostics = [], [], {}, []',
                     'rows, common, provenance, diagnostics, backend_rows, backend_common = [], [], {}, [], [], []')
    anchor = "    T = rows[0]['frames']\n"
    extra = """        if cohort == 'development':
            earlier = OUT.parent / 'icra_evidence_iteration'
            summary = json.loads((earlier / 'summary_development.json').read_text())
            old_path = earlier / 'results_development' / f'{name}_{condition}.json.gz'
            entry = next(r for r in summary['inputs'] if r['sequence'] == name and r['condition'] == condition)
            assert sha(old_path) == entry['sha256']
            old_data = read(old_path)
            for key in ['time', 'positions', 'truth', 'truthIds', 'delivered']:
                assert data[key] == old_data[key], (name, condition, 'unmarked interaction inputs', key)
            provenance[str(old_path.relative_to(ROOT))] = sha(old_path)
            for candidate in ARMS:
                old_arm = candidate.removeprefix('marked_')
                old_run = next(r for r in old_data['runs'] if r['arm'] == old_arm)
                old_row, old_matches = score_run(old_data, old_run)
                expected = next(r for r in summary['runs'] if r['sequence'] == name and r['condition'] == condition and r['arm'] == old_arm)
                for key in METRICS[:6]:
                    assert np.isclose(old_row[key], expected[key], atol=1e-10, rtol=0)
                backend_rows.append(dict(sequence=name, condition=condition, arm=old_arm, frames=len(data['time']), **old_row))
                matches = candidate_matches[candidate]
                both = np.isfinite(matches) & np.isfinite(old_matches)
                backend_common.append(dict(sequence=name, condition=condition, candidate=candidate, reference=old_arm,
                                           support=int(both.sum()), candidate_sse=float(matches[both].sum()),
                                           reference_sse=float(old_matches[both].sum())))
"""
    source = replace(source, anchor, extra + anchor)
    source = replace(source, 'diagnostics=diagnostics, input_sha256=provenance,',
                     'diagnostics=diagnostics, backend_runs=backend_rows, backend_common=backend_common, input_sha256=provenance,')
    source = replace(source, "    (OUT / f'summary_{cohort}.json').write_text",
                     """    result['unmarked_backend_runs'] = [r for report in reports for r in report['backend_runs']]
    result['rescored_unmarked_backend_node_frames'] = 4 * len(ARMS) * result['frames'] if cohort == 'development' else 0
    result['backend_interaction'] = []
    if cohort == 'development':
        old_lookup = {(r['sequence'], r['condition'], r['arm']): r for r in result['unmarked_backend_runs']}
        for condition in frozen['conditions']:
            for candidate in ARMS:
                old_arm = candidate.removeprefix('marked_')
                a = [lookup[name, condition, candidate] for name in names]
                b = [old_lookup[name, condition, old_arm] for name in names]
                group = [r for report in reports for r in report['backend_common'] if r['condition'] == condition and r['candidate'] == candidate]
                support = sum(r['support'] for r in group)
                result['backend_interaction'].append(dict(condition=condition, candidate=candidate, reference=old_arm,
                    **{key: interval([x[key]-y[key] for x,y in zip(a,b)], samples) for key in METRICS},
                    common_support=support, candidate_rmse=float(np.sqrt(sum(r['candidate_sse'] for r in group)/support)),
                    reference_rmse=float(np.sqrt(sum(r['reference_sse'] for r in group)/support))))
    (OUT / f'summary_{cohort}.json').write_text""")
    (OUT / 'analyze_marked_joint.py').write_text(source)
    print('Marked joint auditor generated with independent local-increment and fusion checks.')


if __name__ == '__main__':
    main()
