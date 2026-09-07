function runReviewerBaselinesN50(outputDir, trialSeeds, runMissingTrials)
% RUNREVIEWERBASELINESN50 Run the two reviewer-requested adaptations.
% Each deterministic seed is saved atomically with a protocol fingerprint.
% Fixed and proposed rows are reused from the matched GOSPA/core run.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(scriptDir));
if nargin < 1 || isempty(outputDir)
    outputDir = fullfile(projectRoot, 'RUN', 'GA', ...
        'reviewer_baselines_validation');
end
if nargin < 2 || isempty(trialSeeds)
    trialSeeds = 2:51;
end
if nargin < 3 || isempty(runMissingTrials)
    runMissingTrials = true;
end

expectedArmNames = {'Zheng-style subdensity GA-LMB adaptation', ...
    'Gao-style local-trust GA-LMB adaptation'};
[aggregate, runMetadata] = runResumableGospaExperiment( ...
    outputDir, 'reviewer_baselines', trialSeeds, runMissingTrials, ...
    'reviewerBaselines', [2, 3], expectedArmNames);
aggregate.startedAt = runMetadata.startedAt;
aggregate.completedAt = runMetadata.completedAt;

aggregatePath = fullfile(outputDir, 'reviewer_baselines_n50_summary.mat');
save('-mat7-binary', aggregatePath, 'aggregate', 'runMetadata');

completionPath = fullfile(outputDir, 'reviewer_baselines_n50_completed.txt');
fid = fopen(completionPath, 'w');
assert(fid >= 0, 'Unable to write completion record: %s', completionPath);
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, 'started_at=%s\n', aggregate.startedAt);
fprintf(fid, 'completed_at=%s\n', aggregate.completedAt);
fprintf(fid, 'summary=%s\n', aggregatePath);
fprintf(fid, 'trial_directory=%s\n', runMetadata.trialDir);
fprintf(fid, 'trials=%d\n', numel(trialSeeds));
fprintf(fid, 'seeds=%s\n', mat2str(trialSeeds));
fprintf(fid, 'gospa_c=5\n');
fprintf(fid, 'gospa_p=2\n');
fprintf(fid, 'gospa_alpha=2\n');
fprintf(fid, 'gospa_ground_space=complete_extracted_kinematic_state_vector\n');
fprintf(fid, 'arms=%s\n', strjoin(aggregate.armNames, ' | '));
fprintf(fid, 'implementations=explicit_adaptations_not_exact_source_reproductions\n');
fprintf(fid, 'reference_rows=Fixed and proposed modes from matched GOSPA/core run\n');

fprintf('REVIEWER_BASELINES_N50_COMPLETE\n');
fprintf('Summary: %s\n', aggregatePath);
end
