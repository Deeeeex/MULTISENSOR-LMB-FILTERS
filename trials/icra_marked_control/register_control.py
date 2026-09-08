"""Fix all additional comparisons before full control-cohort execution."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]
PORT = OUT.parent / 'icra_fusion_holdout'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    target = OUT / 'CONTROL_FREEZE.json'
    assert not target.exists(), 'Preserve the registered control scope.'
    preflight = json.loads((OUT / 'preflight_audit.json').read_text())
    assert preflight['passed'] and preflight['er_exact_node_frames'] == 588
    assert preflight['auditor_sha256'] == sha(OUT / 'analyze_control.py')
    for name, expected in preflight['input_sha256'].items():
        assert sha(ROOT / name) == expected
    source = json.loads((OUT / 'source_sha256.json').read_text())
    for name, expected in source.items():
        assert sha(ROOT / name) == expected, name
    primary = json.loads((PORT / 'METHOD_FREEZE.json').read_text())
    ledger = json.loads((PORT / 'holdout_runtime.json').read_text())
    inputs = [OUT.parent / 'icra_external_fusion/v2v4real_input_manifest.json', PORT / 'input_manifest.json',
              OUT.parent / 'icra_ceiling_iteration/calibration.json',
              OUT.parent / 'icra_marked_iteration/likelihood_manifest.json',
              OUT.parent / 'icra_marked_iteration/summary_development.json']
    evidence = inputs + [OUT / 'source_sha256.json', OUT / 'preflight_audit.json', OUT / 'PROTOCOL.md',
                         OUT / 'analyze_control.py', OUT / 'run_control.py', Path(__file__),
                         PORT / 'METHOD_FREEZE.json', PORT / 'analyze_holdout.py']
    result = dict(protocol='shared-information-conservative-control-v1',
                  timestamp_utc=datetime.now(timezone.utc).isoformat(),
                  primary='marked_ceiling_score', additional_control='marked_conservative',
                  full_primary_cohort_aggregate_exists=(PORT / 'summary_holdout.json').exists(),
                  completed_primary_sequences_at_registration=[r['sequence'] for r in ledger
                      if r['returncode'] == 0 and r['completion_line'] and r['files'] == 32],
                  conditions=primary['conditions'], bootstrap=primary['bootstrap'],
                  added_after_partial_primary_outputs=True, no_primary_reselection=True,
                  units=[dict(cohort='development', sequence=s, index=s) for s in range(9)] +
                        [dict(cohort='holdout', sequence=s, index=i) for i, s in enumerate(primary['units'])],
                  source_and_evidence_sha256={str(p.relative_to(ROOT)): sha(p) for p in evidence})
    assert not result['full_primary_cohort_aggregate_exists']
    assert len(result['units']) == 34
    target.write_text(json.dumps(result, indent=2) + '\n')
    print('Extra conservative control fixed for all 34 sequences, both conditions; original primary unchanged.', flush=True)


if __name__ == '__main__':
    main()
