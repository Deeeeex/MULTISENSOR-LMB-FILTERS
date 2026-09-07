function runGospaCoreN50(outputDir, trialSeeds, runMissingTrials)
% RUNGOSPACOREN50 Re-evaluate four core arms with resumable GOSPA trials.
% Every deterministic seed is saved atomically.  Existing seed files are
% validated and skipped, so an interrupted N=50 run resumes safely.

scriptDir = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(scriptDir));
if nargin < 1 || isempty(outputDir)
    outputDir = fullfile(projectRoot, 'RUN', 'GA', 'gospa_validation');
end
if nargin < 2 || isempty(trialSeeds)
    trialSeeds = 2:51;
end
if nargin < 3 || isempty(runMissingTrials)
    runMissingTrials = true;
end

expectedArmNames = {'fixed weights', 'Cao-Zhao FID-FIA baseline', ...
    '+structure-aware decoupled KLA', '+FID-FIA existence refinement'};
[summary, runMetadata] = runResumableGospaExperiment( ...
    outputDir, 'gospa_core', trialSeeds, runMissingTrials, ...
    'fidFiaExistenceRefinement', 1:4, expectedArmNames);
legacyTrialAudit = validateGospaCoreAgainstLegacy(summary);

summaryPath = fullfile(outputDir, 'gospa_core_n50_summary.mat');
startedAt = runMetadata.startedAt;
completedAt = runMetadata.completedAt;
save('-mat7-binary', summaryPath, 'summary', 'startedAt', 'completedAt', ...
    'runMetadata', 'legacyTrialAudit');

completionPath = fullfile(outputDir, 'gospa_core_n50_completed.txt');
fid = fopen(completionPath, 'w');
assert(fid >= 0, 'Unable to write completion record: %s', completionPath);
cleanupObj = onCleanup(@() fclose(fid));
fprintf(fid, 'started_at=%s\n', startedAt);
fprintf(fid, 'completed_at=%s\n', completedAt);
fprintf(fid, 'summary=%s\n', summaryPath);
fprintf(fid, 'trial_directory=%s\n', runMetadata.trialDir);
fprintf(fid, 'trials=%d\n', numel(trialSeeds));
fprintf(fid, 'seeds=%s\n', mat2str(trialSeeds));
fprintf(fid, 'gospa_c=5\n');
fprintf(fid, 'gospa_p=2\n');
fprintf(fid, 'gospa_alpha=2\n');
fprintf(fid, 'gospa_ground_space=complete_extracted_kinematic_state_vector\n');
fprintf(fid, 'arms=%s\n', strjoin(summary.armNames, ' | '));
fprintf(fid, 'legacy_per_seed_validation=passed\n');
fprintf(fid, 'legacy_max_ospa_delta=%.12g\n', legacyTrialAudit.maxOspaDelta);
fprintf(fid, 'legacy_max_localization_delta=%.12g\n', ...
    legacyTrialAudit.maxLocalizationDelta);
fprintf(fid, 'legacy_max_cardinality_delta=%.12g\n', ...
    legacyTrialAudit.maxCardinalityDelta);
assert(legacyTrialAudit.localAggregateChecked, ...
    'Full core run did not execute the local aggregate gate.');
fprintf(fid, 'legacy_local_aggregate_validation=passed\n');
fprintf(fid, 'legacy_max_local_aggregate_delta=%.12g\n', ...
    legacyTrialAudit.maxLocalAggregateDelta);

fprintf('GOSPA_CORE_N50_COMPLETE\n');
fprintf('Summary: %s\n', summaryPath);
end
