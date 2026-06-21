% RUNAAHYBRIDVSGAN50VALIDATION
% Paired N=50 validation for Tuned spatial-KLA AA against the two GA
% reference modes on the same 24-step diagnostic scenario.

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

numberOfTrials = 50;
baseSeed = 1;

fprintf('AA tuned hybrid vs GA N%d validation started at %s\n', ...
    numberOfTrials, datestr(now, 'yyyy-mm-dd HH:MM:SS'));
fprintf('Repo: %s\n', projectRoot);
fprintf('Commit: ');
system('git rev-parse HEAD');
fflush(stdout);

fprintf('\n=== GA references: Balanced and FID-FIA final, N=%d, lifeSpan=24 ===\n', numberOfTrials);
fflush(stdout);
[gaReportPath, gaSummary] = runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
    numberOfTrials, baseSeed, true, struct(), true, 'fidFiaExistenceRefinement', ...
    struct(), [3 4], struct('targetFormationLifeSpan', 24));
fprintf('GA_N50_REPORT=%s\n', gaReportPath);
disp(gaSummary.consensus);
disp(gaSummary.local.meanAcrossSensors);
fflush(stdout);

fprintf('\n=== AA tuned hybrid: Tuned spatial-KLA AA, threshold=0.18, N=%d ===\n', numberOfTrials);
fflush(stdout);
[aaReportPath, aaSummary] = runAaBalancedCardinalityValidation( ...
    numberOfTrials, baseSeed, true, ...
    struct('saveMat', false, 'saveCheckpoints', false, ...
        'progressEverySteps', 0, 'existenceThreshold', 0.18), ...
    struct(), true, [9]);
fprintf('AA_TUNED_HYBRID_N50_REPORT=%s\n', aaReportPath);
disp(aaSummary.consensus);
disp(aaSummary.local.meanAcrossSensors);
fflush(stdout);

fprintf('\nAA tuned hybrid vs GA N%d validation finished at %s\n', ...
    numberOfTrials, datestr(now, 'yyyy-mm-dd HH:MM:SS'));
