% RUNAAHYBRIDVSGAN10VALIDATION
% Paired N=10 validation for the current AA-existence + KLA-spatial winner
% against the two GA paper modes on the same 24-step diagnostic scenario.

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
addpath(fullfile(projectRoot, 'RUN', 'GA'));
setPath;

fprintf('AA hybrid vs GA N10 validation started at %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf('Repo: %s\n', projectRoot);
fprintf('Commit: ');
system('git rev-parse HEAD');
fflush(stdout);

fprintf('\n=== GA references: Balanced and FID-FIA final, N=10, lifeSpan=24 ===\n');
fflush(stdout);
[gaReportPath, gaSummary] = runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
    10, 1, true, struct(), true, 'fidFiaExistenceRefinement', ...
    struct(), [3 4], struct('targetFormationLifeSpan', 24));
fprintf('GA_N10_REPORT=%s\n', gaReportPath);
disp(gaSummary.consensus);
disp(gaSummary.local.meanAcrossSensors);
fflush(stdout);

fprintf('\n=== AA N1 winner: Balanced spatial-KLA AA, threshold=0.05, N=10 ===\n');
fflush(stdout);
[aaReportPath, aaSummary] = runAaBalancedCardinalityValidation( ...
    10, 1, true, ...
    struct('saveMat', false, 'saveCheckpoints', false, ...
        'progressEverySteps', 0, 'existenceThreshold', 0.05), ...
    struct(), true, [7]);
fprintf('AA_HYBRID_N10_REPORT=%s\n', aaReportPath);
disp(aaSummary.consensus);
disp(aaSummary.local.meanAcrossSensors);
fflush(stdout);

fprintf('\nAA hybrid vs GA N10 validation finished at %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
