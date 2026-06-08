function [reportPath, summaryPath, summary] = ...
    runFidFiaExistenceNoStabilizationValidation(numberOfTrials, baseSeed)
% RUNFIDFIAEXISTENCENOSTABILIZATIONVALIDATION
% Validate Balanced + existence-only FID-FIA without EMA or weight floors.
%
% The default run uses 50 deterministic seeds (2:51). Only the FID-FIA
% extension arm is executed. The report contains network disagreement,
% local tracking metrics, trial variability, and runtime.
%
% MATLAB/Octave:
%   addpath('RUN/GA');
%   [reportPath, summaryPath, summary] = ...
%       runFidFiaExistenceNoStabilizationValidation();

if nargin < 1 || isempty(numberOfTrials)
    numberOfTrials = 50;
end
if nargin < 2 || isempty(baseSeed)
    baseSeed = 1;
end

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end
projectRoot = resolveProjectRoot(scriptDir);
addpath(projectRoot);
addpath(scriptDir);

[generatedReportPath, summary] = ...
    runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
        numberOfTrials, baseSeed, true, struct(), true, ...
        'fidFiaExistenceRefinement', struct(), 4);

assert(numel(summary.arms) == 1);
assert(strcmp(summary.armNames{1}, '+FID-FIA existence refinement'));
assert(summary.arms(1).adaptiveFusion.useFidFiaExistence);
assertNoStabilization(summary.arms(1).adaptiveFusion);

timestamp = datestr(now, 'yyyymmdd_HHMMSS');
reportPath = fullfile(scriptDir, sprintf( ...
    'GA_FID_FIA_EXISTENCE_NO_STABILIZATION_N%d_SEED%d_%s.md', ...
    numberOfTrials, baseSeed, timestamp));
if ~strcmp(generatedReportPath, reportPath)
    movefile(generatedReportPath, reportPath);
end

summaryPath = fullfile(scriptDir, sprintf( ...
    'GA_FID_FIA_EXISTENCE_NO_STABILIZATION_N%d_SEED%d_%s.mat', ...
    numberOfTrials, baseSeed, timestamp));
save(summaryPath, 'summary', 'reportPath');

fprintf('\nBalanced + FID-FIA existence without EMA/floor\n');
fprintf('Network disagreement: OSPA %.6f, Loc %.6f, Card %.6f\n', ...
    summary.consensus.ospa(1), summary.consensus.pos(1), ...
    summary.consensus.card(1));
fprintf('Local metrics: E-OSPA %.6f, RMSE %.6f, CardErr %.6f\n', ...
    summary.local.meanAcrossSensors.eOspa(1), ...
    summary.local.meanAcrossSensors.rmse(1), ...
    summary.local.meanAcrossSensors.cardErr(1));
fprintf('Report: %s\n', reportPath);
fprintf('Summary: %s\n', summaryPath);
end

function assertNoStabilization(cfg)
fields = {'emaAlpha', 'minWeight', 'spatialEmaAlpha', ...
    'existenceEmaAlpha', 'spatialMinWeight', 'existenceMinWeight'};
for fieldIdx = 1:numel(fields)
    fieldName = fields{fieldIdx};
    assert(isfield(cfg, fieldName) && cfg.(fieldName) == 0.0, ...
        'FID-FIA extension unexpectedly enables %s.', fieldName);
end
end

function projectRoot = resolveProjectRoot(scriptDir)
projectRoot = scriptDir;
while ~exist(fullfile(projectRoot, 'setPath.m'), 'file')
    parentDir = fileparts(projectRoot);
    if strcmp(parentDir, projectRoot)
        error('Could not locate project root from %s.', scriptDir);
    end
    projectRoot = parentDir;
end
end
