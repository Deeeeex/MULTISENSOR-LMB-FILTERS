"""Collect complete reviewer experiments into a portable evidence snapshot."""
from pathlib import Path
import hashlib
import json
import shutil

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
NATIVE = ROOT / 'trials/icra_reviewer_revision'
DATA = HERE / 'source_data/reviewer_revision'
MANIFEST = HERE / 'source_data/reviewer_source_manifest.json'
SOURCES = [
    'CONTROLS_ANALYSIS.json', 'NEW_DATA_ANALYSIS.json', 'MODEL_SENSITIVITY_ANALYSIS.json',
    'LEGACY_RESCORING.json', 'FIXED_SELECTION.json', 'correlation_control.json',
    'CORRELATION_AUDIT_V2.json', 'OFFICIAL_ADAPTER_PREFLIGHT.json', 'NEW_INPUT_AUDIT.json',
    'NEW_INPUT_MANIFEST.json', 'NEW_DETECTION_MANIFEST.json', 'NEW_DATA_FREEZE.json',
    'POSE_INPUTS_development.json', 'POSE_INPUTS_seen_transfer.json',
    'audit_controls_development.json', 'audit_gs_seen_transfer.json',
    'audit_fixed025_seen_transfer.json', 'audit_new_validation_primary.json',
    'audit_new_validation_recency.json', 'audit_motion_development.json',
    'audit_motion_seen_transfer.json', 'audit_motion_new_validation.json',
    'audit_pd070_development.json', 'audit_pd080_development.json', 'audit_pd095_development.json',
    'audit_preflight_v1.json', 'audit_motion_identity_preflight.json', 'RECENCY_PREFLIGHT_AUDIT.json',
    'PROTOCOL.md', 'CANCELLED_STAGES.json', 'PRE_REVISION_FREEZE.json', 'DATA_DIRECTORY_AUDIT.json',
    'REPLAY_ACCEPTANCE.json', 'RESULT_FILES_MANIFEST.json',
]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if NATIVE.is_dir():
        assert all((NATIVE / name).is_file() for name in SOURCES), 'Reviewer experiments are incomplete'
        DATA.mkdir(parents=True, exist_ok=True)
        manifest = {}
        for name in SOURCES:
            source, target = NATIVE / name, DATA / name
            if name.startswith('audit_') or name in ['CONTROLS_ANALYSIS.json', 'NEW_DATA_ANALYSIS.json',
                    'MODEL_SENSITIVITY_ANALYSIS.json', 'LEGACY_RESCORING.json', 'CORRELATION_AUDIT_V2.json',
                    'OFFICIAL_ADAPTER_PREFLIGHT.json', 'NEW_INPUT_AUDIT.json', 'RECENCY_PREFLIGHT_AUDIT.json']:
                assert json.loads(source.read_text())['passed'], name
            shutil.copyfile(source, target)
            manifest[name] = dict(source=str(source.relative_to(ROOT)), sha256=sha(target))
        MANIFEST.write_text(json.dumps(manifest, indent=2) + '\n')
    else:
        manifest = json.loads(MANIFEST.read_text())
    assert sorted(manifest) == sorted(SOURCES)
    for name, record in manifest.items():
        assert sha(DATA / name) == record['sha256'], name
    # A report cannot silently refer to a different result snapshot.
    for name, key in [('CONTROLS_ANALYSIS.json', 'sources'), ('NEW_DATA_ANALYSIS.json', 'source_reports'),
                      ('MODEL_SENSITIVITY_ANALYSIS.json', 'source_reports')]:
        value = json.loads((DATA / name).read_text())
        for relative, digest in value[key].items():
            if relative in manifest:
                assert manifest[relative]['sha256'] == digest, (name, relative)
    evidence = dict(
        controls=json.loads((DATA / 'CONTROLS_ANALYSIS.json').read_text()),
        new_data=json.loads((DATA / 'NEW_DATA_ANALYSIS.json').read_text()),
        sensitivity=json.loads((DATA / 'MODEL_SENSITIVITY_ANALYSIS.json').read_text()),
        correlation=json.loads((DATA / 'correlation_control.json').read_text()),
        source_manifest_sha256=sha(MANIFEST), generator_sha256=sha(Path(__file__)),
        scope='Complete audited experiments, including unfavorable outcomes. Portable rows reproduce tables and statistical summaries; native posterior traces and raw public sensor files remain in the research checkout.')
    (HERE / 'source_data/reviewer_evidence.json').write_text(json.dumps(evidence, indent=2, allow_nan=False) + '\n')
    print('Collected', len(manifest), 'complete reviewer evidence snapshots.')


if __name__ == '__main__':
    main()
