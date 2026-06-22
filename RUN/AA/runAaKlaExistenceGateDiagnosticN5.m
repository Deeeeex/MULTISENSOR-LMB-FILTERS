% RUNAAKLAEXISTENCEGATEDIAGNOSTICN5
% Focused N=5 diagnostic for the remaining N50 consensus-Loc gap.
%
% It compares the current Tuned spatial-KLA AA arm against a target-wise
% existence-gated KLA-spatial variant on the seed block that showed elevated
% N50 Loc disagreement.  Weight diagnostics are enabled only for this runner.

close all; clc;
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end
projectRoot = scriptDir;
for k = 1:6
    if exist(fullfile(projectRoot, 'setPath.m'), 'file')
        break;
    end
    parent = fileparts(projectRoot);
    if isempty(parent) || strcmp(parent, projectRoot)
        break;
    end
    projectRoot = parent;
end
cd(projectRoot);
addpath(projectRoot);
addpath(fullfile(projectRoot, 'RUN', 'AA'));
setPath;

numberOfTrials = 5;
baseSeed = 11;
armSelection = [9 10];
aaControls = struct( ...
    'saveMat', false, ...
    'saveCheckpoints', false, ...
    'progressEverySteps', 0, ...
    'existenceThreshold', 0.18);
adaptiveFusionOverrides = struct('captureWeightDiagnostics', true);

fprintf('AA KLA existence-gate diagnostic N%d baseSeed=%d started at %s\n', ...
    numberOfTrials, baseSeed, datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf('Repo: %s\n', projectRoot);
fprintf('Commit: ');
system('git rev-parse HEAD');
fflush(stdout);

[reportPath, summary] = runAaBalancedCardinalityValidation( ...
    numberOfTrials, baseSeed, true, aaControls, struct(), true, ...
    armSelection, adaptiveFusionOverrides);

fprintf('AA_KLA_EXISTENCE_GATE_DIAGNOSTIC_REPORT=%s\n', reportPath);
disp(summary.consensus);
disp(summary.local.meanAcrossSensors);
fflush(stdout);

fprintf('AA KLA existence-gate diagnostic N%d baseSeed=%d finished at %s\n', ...
    numberOfTrials, baseSeed, datestr(now, 'yyyy-mm-dd HH:MM:SS'));
