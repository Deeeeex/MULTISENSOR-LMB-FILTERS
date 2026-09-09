"""Add an audited coordinate adapter to an immutable copy of the v1 runner."""
from pathlib import Path
import hashlib
import json

OUT = Path(__file__).resolve().parent


def main():
    source = OUT / 'runReviewerReplay.m'; destination = OUT / 'runMotionReviewerReplay.m'
    assert not destination.exists()
    code = source.read_text()
    edits = [
        ('function runReviewerReplay(configPath,unitIndex)', 'function runMotionReviewerReplay(configPath,unitIndex)'),
        ('checkStableMarkedEvidence();checkGaussianEvidence();checkReviewerEvidence();',
         'checkStableMarkedEvidence();checkGaussianEvidence();checkReviewerEvidence();checkReviewerEgoMotion();'),
        ('model=makeModel(positions,T,baseQuality,config.pd);',
         "model=makeModel(positions,T,baseQuality,config.pd);\nmotion=repmat(eye(3),1,1,T);\nif strcmp(config.coordinate_adapter,'planar')\n    pose=load(fullfile(root,unit.pose_path));motion=pose.egoPrevToCurrent;\n    assert(isequal(size(motion),[3,3,T]));\nelse\n    assert(any(strcmp(config.coordinate_adapter,{'none','identity'})));\nend"),
        ('run=runDensity(model,measurements,positions,delivered,arm,marks,ratios);',
         'run=runDensity(model,measurements,positions,delivered,arm,marks,ratios,motion);'),
        ("'protocol','icra-reviewer-revision-v1','stage',config.stage", "'protocol','icra-reviewer-revision-v2','stage',config.stage"),
        ("'pd',config.pd,'time',double(data.time),'positions',positions", "'pd',config.pd,'coordinateAdapter',config.coordinate_adapter,'egoPrevToCurrent',motion,'time',double(data.time),'positions',positions"),
        ('function run=runDensity(model,measurements,positions,delivered,arm,marks,ratios)',
         'function run=runDensity(model,measurements,positions,delivered,arm,marks,ratios,motion)'),
        ('        predictedPd=zeros(1,numel(predicted));',
         '        % Transform both surviving predictions and previous-frame births.\n        predicted=applyReviewerEgoMotion(predicted,motion(:,:,t));\n        predictedPd=zeros(1,numel(predicted));')]
    for old, new in edits:
        assert code.count(old) == 1, old
        code = code.replace(old, new)
    destination.write_text(code)
    (OUT / 'MOTION_RUNNER_PATCH.json').write_text(json.dumps(dict(
        base_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        generated_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(), replacements=edits), indent=2) + '\n')
    print('GENERATED isolated motion/new-cohort runner with',len(edits),'bounded replacements')


if __name__ == '__main__':
    main()
