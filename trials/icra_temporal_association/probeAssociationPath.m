function probeAssociationPath()
% Isolated recursive controls on immutable, externally registered inputs.
out=fileparts(mfilename('fullpath'));root=fileparts(fileparts(out));

original=fullfile(root,'trials','icra_external_fusion');
addpath(fullfile(root,'trials','icra_reviewer_revision'));
priorIteration=fullfile(root,'trials','icra_method_iteration');
folders={'icra_fusion_holdout','icra_asymmetric_evidence','icra_selective_innovation', ...
    'icra_evidence_iteration','icra_ceiling_iteration','icra_marked_iteration', ...
    'icra_gaussian_evidence','icra_reunion_fusion'};
for k=1:numel(folders),addpath(fullfile(root,'trials',folders{k}));end
addpath(original,priorIteration,fullfile(root,'common'),fullfile(root,'lmb'),fullfile(root,'multisensorLmb'));
checkStableMarkedEvidence();checkGaussianEvidence();checkReviewerEvidence();checkDirectObservationSummary();
baseQuality=@evaluateSensorQuality;
assert(strcmp(functions(baseQuality).file,fullfile(root,'common','evaluateSensorQuality.m')));
qualityPath=fullfile(original,'replay_quality');addpath(qualityPath);
cleanup=onCleanup(@()rmpath(qualityPath)); %#ok<NASGU>
runtimePath=fullfile(priorIteration,'runtime');addpath(runtimePath);
runtimeCleanup=onCleanup(@()rmpath(runtimePath)); %#ok<NASGU>
assert(strcmp(which('Hungarian'),fullfile(runtimePath,'Hungarian.m')));
fprintf('QUALITY BEFORE UNIT %s\n',which('evaluateSensorQuality'));
checkPersistentAssociation();
fprintf('QUALITY AFTER UNIT %s\n',which('evaluateSensorQuality'));
addpath(qualityPath,'-begin');
fprintf('QUALITY RESTORED %s\n',which('evaluateSensorQuality'));
assert(strcmp(which('evaluateSensorQuality'),fullfile(qualityPath,'evaluateSensorQuality.m')));
end
