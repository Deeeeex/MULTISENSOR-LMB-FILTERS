"""Build validated caches out of order; keep cohort decisions in the fixed auditor."""
from pathlib import Path
import json
import analyze_gaussian as audit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[1]


def main():
    frozen = json.loads((OUT / 'ROUND_FREEZE.json').read_text())
    for name, expected in frozen['source_and_evidence_sha256'].items():
        assert audit.sha(ROOT / name) == expected, name
    source = audit.source_check()
    cohort = 'seen_transfer'
    audit.ARMS = frozen['arms_by_cohort'][cohort]
    audit.REFERENCES = frozen['references_by_cohort'][cohort]
    runtime = json.loads((OUT / f'runtime_{cohort}.json').read_text())
    complete = {r['sequence'] for r in runtime if r['returncode'] == 0 and r['completion_line']
                and r['files'] == 2*len(audit.ARMS)}
    cache = OUT / 'audited_sequences'
    count = 0
    for unit in frozen['units']:
        if unit['cohort'] != cohort or unit['sequence'] not in complete:
            continue
        path = cache / f"{cohort}_{unit['sequence']:04d}.json"
        if path.exists():
            continue
        result = audit.audit_unit(unit, source)
        temporary = path.with_suffix('.additional.next')
        temporary.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
        temporary.replace(path)
        count += 1
        print('ADDITIONAL GAUSSIAN SEQUENCE AUDITED', unit['sequence'], result['new_node_frames'], flush=True)
    print('Additional complete sequence caches:', count, '; no cohort statistics before all 25.', flush=True)


if __name__ == '__main__':
    main()
