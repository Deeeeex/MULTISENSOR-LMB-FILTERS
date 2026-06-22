% RUNAALOCFAILUREATTRIBUTIONN5
% Target/time attribution for the remaining Tuned spatial-KLA AA Loc gap.
%
% This runner keeps the final Tuned spatial-KLA AA configuration and records
% the highest per-time consensus-Loc disagreement rows on the seeds 12-16
% block.  It is diagnostic-only and does not introduce a new method arm.

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
armSelection = [9];
aaControls = struct( ...
    'saveMat', false, ...
    'saveCheckpoints', false, ...
    'progressEverySteps', 0, ...
    'existenceThreshold', 0.18, ...
    'captureConsensusAttribution', true, ...
    'consensusAttributionTopK', 5);
adaptiveFusionOverrides = struct('captureWeightDiagnostics', true);

fprintf('AA Loc failure attribution N%d baseSeed=%d started at %s\n', ...
    numberOfTrials, baseSeed, datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf('Repo: %s\n', projectRoot);
fprintf('Commit: ');
system('git rev-parse HEAD');
fflush(stdout);

[reportPath, summary] = runAaBalancedCardinalityValidation( ...
    numberOfTrials, baseSeed, true, aaControls, struct(), true, ...
    armSelection, adaptiveFusionOverrides);

fprintf('AA_LOC_FAILURE_ATTRIBUTION_REPORT=%s\n', reportPath);
disp(summary.consensus);
disp(summary.local.meanAcrossSensors);
fflush(stdout);

fprintf('AA Loc failure attribution N%d baseSeed=%d finished at %s\n', ...
    numberOfTrials, baseSeed, datestr(now, 'yyyy-mm-dd HH:MM:SS'));
