% RUNAATUNEDDESIGNABLATIONN50VALIDATION
% N=50 ablation for the AA-existence + KLA-spatial hybrid design.
%
% The comparison keeps the same 24-step diagnostic scenario and
% existenceThreshold=0.18 as the tuned N50 gate, then isolates:
%   1) pure AA spatial mixture,
%   2) spatial-KLA hybrid without the final spatial retuning,
%   3) FID-FIA existence refinement on the hybrid path,
%   4) the final tuned spatial-KLA AA configuration.

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

numberOfTrials = 50;
baseSeed = 1;
armSelection = [3 7 8 9];
aaControls = struct( ...
    'saveMat', false, ...
    'saveCheckpoints', false, ...
    'progressEverySteps', 0, ...
    'existenceThreshold', 0.18);

fprintf('AA tuned design N%d ablation started at %s\n', ...
    numberOfTrials, datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf('Repo: %s\n', projectRoot);
fprintf('Commit: ');
system('git rev-parse HEAD');
fflush(stdout);

[reportPath, summary] = runAaBalancedCardinalityValidation( ...
    numberOfTrials, baseSeed, true, aaControls, struct(), true, armSelection);

fprintf('AA_TUNED_DESIGN_ABLATION_N50_REPORT=%s\n', reportPath);
disp(summary.consensus);
disp(summary.local.meanAcrossSensors);
fflush(stdout);

fprintf('AA tuned design N%d ablation finished at %s\n', ...
    numberOfTrials, datestr(now, 'yyyy-mm-dd HH:MM:SS'));
