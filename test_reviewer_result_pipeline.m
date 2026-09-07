function test_reviewer_result_pipeline()
% TEST_REVIEWER_RESULT_PIPELINE Fast regression test for revision artifacts.
% No filtering is executed. Synthetic paired summaries exercise protocol
% rejection, resumable collection, and paper-facing table generation.

addpath(fullfile(pwd, 'RUN', 'GA'));
testRoot = tempname();
mkdir(testRoot);
cleanupObj = onCleanup(@() rmdir(testRoot, 's'));

fprintf('Test 1: resumable GOSPA protocol guard and skip path\n');
resumeDir = fullfile(testRoot, 'resume');
mkdir(resumeDir);
trialDir = fullfile(resumeDir, 'gospa_core_trials');
mkdir(trialDir);
trialSeed = 2;
expectedCore = {'fixed weights', 'Cao-Zhao FID-FIA baseline', ...
    '+structure-aware decoupled KLA', '+FID-FIA existence refinement'};
protocol = makeProtocol('fidFiaExistenceRefinement', 1:4, expectedCore);
summary = makeCoreSummary(1, trialSeed);
trialStartedAt = 'synthetic';
trialCompletedAt = 'synthetic';
trialPath = fullfile(trialDir, 'gospa_core_seed_002.mat');
save('-mat7-binary', trialPath, 'summary', 'trialSeed', 'protocol', ...
    'trialStartedAt', 'trialCompletedAt');

[collected, metadata] = runResumableGospaExperiment( ...
    resumeDir, 'gospa_core', trialSeed, false, ...
    'fidFiaExistenceRefinement', 1:4, expectedCore);
assert(isequal(metadata.skippedSeeds, trialSeed));
assert(isempty(metadata.executedSeeds));
assert(isequal(collected.trialSeeds, trialSeed));
assert(abs(collected.consensus.ospa(1) - 2.468973) < 1e-12);

protocol.gospaGroundSpace = 'position_only_invalid';
save('-mat7-binary', trialPath, 'summary', 'trialSeed', 'protocol', ...
    'trialStartedAt', 'trialCompletedAt');
rejected = false;
try
    runResumableGospaExperiment(resumeDir, 'gospa_core', trialSeed, false, ...
        'fidFiaExistenceRefinement', 1:4, expectedCore);
catch err
    rejected = ~isempty(strfind(err.message, 'Metric protocol mismatch')); %#ok<STREMP>
end
assert(rejected, 'A stale ground-space fingerprint was not rejected.');

protocol = makeProtocol('fidFiaExistenceRefinement', 1:4, expectedCore);
summary = makeCoreSummary(1, trialSeed);
summary.localTrials.eOspa(1, 1, 1) = Inf;
save('-mat7-binary', trialPath, 'summary', 'trialSeed', 'protocol', ...
    'trialStartedAt', 'trialCompletedAt');
rejected = false;
try
    runResumableGospaExperiment(resumeDir, 'gospa_core', trialSeed, false, ...
        'fidFiaExistenceRefinement', 1:4, expectedCore);
catch err
    rejected = ~isempty(strfind(err.message, ...
        'Non-finite local eOspa value')); %#ok<STREMP>
end
assert(rejected, 'A non-finite local metric was not rejected.');

summary = makeCoreSummary(1, trialSeed);
summary.commConfig.level = 3;
save('-mat7-binary', trialPath, 'summary', 'trialSeed', 'protocol', ...
    'trialStartedAt', 'trialCompletedAt');
rejected = false;
try
    runResumableGospaExperiment(resumeDir, 'gospa_core', trialSeed, false, ...
        'fidFiaExistenceRefinement', 1:4, expectedCore);
catch err
    rejected = ~isempty(strfind(err.message, ...
        'Unexpected communication level')); %#ok<STREMP>
end
assert(rejected, 'A main-scenario configuration mismatch was not rejected.');

fprintf('Test 2: split seed matches the canonical paired report\n');
legacySeed = makeLegacySeed2Summary();
legacyAudit = validateGospaCoreAgainstLegacy(legacySeed);
assert(isequal(legacyAudit.checkedSeeds, 2));
assert(legacyAudit.maxOspaDelta == 0);
legacySeed.consensusTrials.ospa(1, 1) = ...
    legacySeed.consensusTrials.ospa(1, 1) + 1e-3;
rejected = false;
try
    validateGospaCoreAgainstLegacy(legacySeed);
catch err
    rejected = ~isempty(strfind(err.message, ...
        'Per-seed OSPA differs from the legacy report')); %#ok<STREMP>
end
assert(rejected, 'A per-seed legacy mismatch was not rejected.');

legacyCoreFull = makeLegacyCoreFullSummary();
legacyCoreFullAudit = validateGospaCoreAgainstLegacy(legacyCoreFull);
assert(legacyCoreFullAudit.localAggregateChecked);
assert(legacyCoreFullAudit.maxLocalAggregateDelta < 1e-12);
legacyCoreFull.localTrials.rmse(:, :, 3) = ...
    legacyCoreFull.localTrials.rmse(:, :, 3) + 1e-3;
rejected = false;
try
    validateGospaCoreAgainstLegacy(legacyCoreFull);
catch err
    rejected = ~isempty(strfind(err.message, ...
        'Core local metrics differ')); %#ok<STREMP>
end
assert(rejected, 'A core local aggregate mismatch was not rejected.');

fprintf('Test 3: split PD-weighted seed matches its canonical paired report\n');
legacyPdSeed = makeLegacyPdSeed2Summary();
legacyPdAudit = validatePdWeightedAgainstLegacy(legacyPdSeed);
assert(isequal(legacyPdAudit.checkedSeeds, 2));
assert(legacyPdAudit.maxOspaDelta == 0);
legacyPdSeed.consensusTrials.pos(1, 1) = ...
    legacyPdSeed.consensusTrials.pos(1, 1) + 1e-3;
rejected = false;
try
    validatePdWeightedAgainstLegacy(legacyPdSeed);
catch err
    rejected = ~isempty(strfind(err.message, ...
        'Per-seed PD-weighted localization differs')); %#ok<STREMP>
end
assert(rejected, 'A per-seed PD-weighted mismatch was not rejected.');

legacyPdFull = makeLegacyPdFullSummary();
legacyPdFullAudit = validatePdWeightedAgainstLegacy(legacyPdFull);
assert(legacyPdFullAudit.localAggregateChecked);
assert(legacyPdFullAudit.maxLocalAggregateDelta < 1e-12);
legacyPdFull.localTrials.eOspa(:, :, 1) = ...
    legacyPdFull.localTrials.eOspa(:, :, 1) + 1e-3;
rejected = false;
try
    validatePdWeightedAgainstLegacy(legacyPdFull);
catch err
    rejected = ~isempty(strfind(err.message, ...
        'PD-weighted local metrics differ')); %#ok<STREMP>
end
assert(rejected, 'A PD-weighted local aggregate mismatch was not rejected.');

fprintf('Test 4: GOSPA main-table artifacts share one aggregate\n');
corePath = fullfile(testRoot, 'core.mat');
pdPath = fullfile(testRoot, 'pd.mat');
summary = makeCoreSummary(50, 2:51);
save('-mat7-binary', corePath, 'summary');
summary = makePdSummary(50, 2:51);
save('-mat7-binary', pdPath, 'summary');
mainOut = fullfile(testRoot, 'main');
buildGospaMainTable(corePath, pdPath, mainOut, true);
assertArtifacts(mainOut, {'gospa_main_table_n50_combined.mat', ...
    'gospa_main_table_n50.csv', 'gospa_main_table_n50_per_trial.csv', ...
    'gospa_main_table_n50.md', 'gospa_main_table_n50_rows.tex'});
mainRows = fileread(fullfile(mainOut, 'gospa_main_table_n50_rows.tex'));
assert(~isempty(strfind(mainRows, [char(92), 'pm']))); %#ok<STREMP>
assert(~isempty(strfind(mainRows, [char(92), 'mathbf']))); %#ok<STREMP>
assert(~isempty(strfind(mainRows, ...
    [char(92), 'newcommand{', char(92), 'GospaMainTableRows}']))); %#ok<STREMP>

fprintf('Test 5: reviewer-baseline network/local table artifacts\n');
externalPath = fullfile(testRoot, 'external.mat');
aggregate = makeExternalSummary(50, 2:51);
save('-mat7-binary', externalPath, 'aggregate');
externalOut = fullfile(testRoot, 'external_out');
buildReviewerBaselineTable(corePath, externalPath, externalOut, true);
assertArtifacts(externalOut, {'reviewer_baseline_table_n50_combined.mat', ...
    'reviewer_baseline_table_n50.csv', ...
    'reviewer_baseline_table_n50_per_trial.csv', ...
    'reviewer_baseline_proposed_vs_adaptations.csv', ...
    'reviewer_baseline_table_n50.md', ...
    'reviewer_baseline_table_n50_network_rows.tex', ...
    'reviewer_baseline_table_n50_local_rows.tex'});
assert(~isempty(strfind(fileread(fullfile(externalOut, ...
    'reviewer_baseline_table_n50_network_rows.tex')), ...
    [char(92), 'pm']))); %#ok<STREMP>
assert(~isempty(strfind(fileread(fullfile(externalOut, ...
    'reviewer_baseline_table_n50_local_rows.tex')), ...
    [char(92), 'pm']))); %#ok<STREMP>
externalNetworkRows = fileread(fullfile(externalOut, ...
    'reviewer_baseline_table_n50_network_rows.tex'));
externalLocalRows = fileread(fullfile(externalOut, ...
    'reviewer_baseline_table_n50_local_rows.tex'));
assert(~isempty(strfind(externalNetworkRows, 'Zheng-style adaptation'))); %#ok<STREMP>
assert(isempty(strfind(externalNetworkRows, ...
    'Zheng-style subdensity GA-LMB adaptation'))); %#ok<STREMP>
assert(~isempty(strfind(externalNetworkRows, [char(92), 'mathbf']))); %#ok<STREMP>
assert(~isempty(strfind(externalNetworkRows, ...
    [char(92), 'newcommand{', char(92), 'ReviewerBaselineNetworkRows}']))); %#ok<STREMP>
assert(~isempty(strfind(externalLocalRows, ...
    [char(92), 'newcommand{', char(92), 'ReviewerBaselineLocalRows}']))); %#ok<STREMP>
externalSummaryCsv = fileread(fullfile(externalOut, ...
    'reviewer_baseline_table_n50.csv'));
assert(~isempty(strfind(externalSummaryCsv, 'separate_adaptation_run'))); %#ok<STREMP>
assert(~isempty(strfind(externalSummaryCsv, 'core_run'))); %#ok<STREMP>
assert(isempty(strfind(externalSummaryCsv, 'runtime_relative_to_fixed'))); %#ok<STREMP>
externalReport = fileread(fullfile(externalOut, ...
    'reviewer_baseline_table_n50.md'));
assert(isempty(strfind(externalReport, '| Runtime |'))); %#ok<STREMP>

badCorePath = fullfile(testRoot, 'bad_core.mat');
summary = makeCoreSummary(50, 2:51);
summary.localTrials.eOspa(:, :, 1) = summary.localTrials.eOspa(:, :, 1) + 1e-3;
save('-mat7-binary', badCorePath, 'summary');
rejected = false;
try
    buildReviewerBaselineTable(badCorePath, externalPath, ...
        fullfile(testRoot, 'bad_external_out'), true);
catch err
    rejected = ~isempty(strfind(err.message, ...
        'Reused core local metrics do not reproduce')); %#ok<STREMP>
end
assert(rejected, 'A reused-core local-metric mismatch was not rejected.');

fprintf('Reviewer result-pipeline tests passed.\n');
end

function protocol = makeProtocol(experimentMode, armSelection, expectedArmNames)
protocol = struct( ...
    'version', 1, ...
    'experimentMode', experimentMode, ...
    'armSelection', reshape(armSelection, 1, []), ...
    'expectedArmNames', {expectedArmNames}, ...
    'gospaC', 5, ...
    'gospaP', 2, ...
    'gospaAlpha', 2, ...
    'gospaGroundSpace', 'complete_extracted_kinematic_state_vector');
end

function summary = makeCoreSummary(numTrials, trialSeeds)
numSensors = 8;
summary.armNames = {'fixed weights', 'Cao-Zhao FID-FIA baseline', ...
    '+structure-aware decoupled KLA', '+FID-FIA existence refinement'};
summary.commConfig = makeCommConfig();
summary.arms = makeCoreArms();
summary.trialSeeds = reshape(trialSeeds, 1, []);
summary.pDropBySensorTrials = zeros(numTrials, numSensors);
summary.consensusTrials.ospa = repmat( ...
    [2.468973, 1.817913, 1.779218, 1.677524], numTrials, 1);
summary.consensusTrials.gospa = repmat([4.0, 3.0, 2.9, 2.8], numTrials, 1);
summary.consensusTrials.pos = repmat( ...
    [2.326034, 1.642846, 1.522191, 1.546107], numTrials, 1);
summary.consensusTrials.card = repmat( ...
    [0.716025, 0.122950, 0.187675, 0.063200], numTrials, 1);
summary.localTrials.eOspa = repeatLocal( ...
    [2.862938, 2.184698, 2.334915, 2.019842], ...
    numTrials, numSensors);
summary.localTrials.hOspa = zeros(numTrials, numSensors, 4);
summary.localTrials.rmse = repeatLocal( ...
    [1.649569, 1.734381, 1.605910, 1.720931], ...
    numTrials, numSensors);
summary.localTrials.cardErr = repeatLocal( ...
    [1.455125, 0.388050, 0.578775, 0.223700], ...
    numTrials, numSensors);
summary.runtime.filterSeconds = repmat([50, 140, 55, 150], numTrials, 1);
end

function summary = makeLegacySeed2Summary()
summary = makeCoreSummary(1, 2);
summary.consensusTrials.ospa = [2.605320, 1.919750, 1.878234, 1.714819];
summary.consensusTrials.pos = [2.258061, 1.703510, 1.439986, 1.478188];
summary.consensusTrials.card = [0.932500, 0.160000, 0.240000, 0.066250];
end

function summary = makeLegacyCoreFullSummary()
summary = makeCoreSummary(50, 2:51);
reportPath = fullfile(pwd, 'RUN', 'GA', ...
    'GA_TIERED_LINK_ABLATION_N50_SEED1_20260528_001743.md');
content = fileread(reportPath);
heading = '## Per-Trial Network Disagreement Metrics';
sectionStart = strfind(content, heading) + numel(heading);
remaining = content(sectionStart:end);
nextHeading = strfind(remaining, sprintf('\n## '));
section = remaining(1:nextHeading(1) - 1);
pattern = ['\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*([^|]+?)\s*\|\s*' ...
    '([-+0-9.eE]+)\s*\|\s*([-+0-9.eE]+)\s*\|\s*' ...
    '([-+0-9.eE]+)\s*\|'];
tokens = regexp(section, pattern, 'tokens');
assert(numel(tokens) == 200);
for rowIdx = 1:numel(tokens)
    row = tokens{rowIdx};
    trialIdx = str2double(row{1});
    assert(str2double(row{2}) == summary.trialSeeds(trialIdx));
    armIdx = find(strcmp(summary.armNames, strtrim(row{3})), 1, 'first');
    assert(~isempty(armIdx));
    summary.consensusTrials.ospa(trialIdx, armIdx) = str2double(row{4});
    summary.consensusTrials.pos(trialIdx, armIdx) = str2double(row{5});
    summary.consensusTrials.card(trialIdx, armIdx) = str2double(row{6});
end
end

function summary = makePdSummary(numTrials, trialSeeds)
summary.armNames = {'PD-weighted GA'};
summary.commConfig = makeCommConfig();
summary.arms = makePdArms();
summary.trialSeeds = reshape(trialSeeds, 1, []);
summary.pDropBySensorTrials = zeros(numTrials, 8);
summary.consensusTrials.ospa = repmat(2.177, numTrials, 1);
summary.consensusTrials.gospa = repmat(3.5, numTrials, 1);
summary.consensusTrials.pos = repmat(1.995, numTrials, 1);
summary.consensusTrials.card = repmat(0.588, numTrials, 1);
summary.localTrials.eOspa = repeatLocal(2.735723, numTrials, 8);
summary.localTrials.rmse = repeatLocal(1.562807, numTrials, 8);
summary.localTrials.cardErr = repeatLocal(1.254925, numTrials, 8);
summary.runtime.filterSeconds = repmat(60, numTrials, 1);
end

function summary = makeLegacyPdSeed2Summary()
summary = makePdSummary(1, 2);
summary.consensusTrials.ospa = 2.390301;
summary.consensusTrials.pos = 1.842123;
summary.consensusTrials.card = 0.856250;
end

function summary = makeLegacyPdFullSummary()
summary = makePdSummary(50, 2:51);
reportPath = fullfile(pwd, 'RUN', 'GA', ...
    'Del_GA_TIERED_LINK_ABLATION_N50_SEED1_20260528_092545.md');
content = fileread(reportPath);
heading = '## Per-Trial Network Disagreement Metrics';
sectionStart = strfind(content, heading) + numel(heading);
remaining = content(sectionStart:end);
nextHeading = strfind(remaining, sprintf('\n## '));
section = remaining(1:nextHeading(1) - 1);
pattern = ['\|\s*(\d+)\s*\|\s*(\d+)\s*\|\s*PD-weighted GA\s*\|\s*' ...
    '([-+0-9.eE]+)\s*\|\s*([-+0-9.eE]+)\s*\|\s*' ...
    '([-+0-9.eE]+)\s*\|'];
tokens = regexp(section, pattern, 'tokens');
assert(numel(tokens) == 50);
for rowIdx = 1:numel(tokens)
    row = tokens{rowIdx};
    trialIdx = str2double(row{1});
    assert(str2double(row{2}) == summary.trialSeeds(trialIdx));
    summary.consensusTrials.ospa(trialIdx, 1) = str2double(row{3});
    summary.consensusTrials.pos(trialIdx, 1) = str2double(row{4});
    summary.consensusTrials.card(trialIdx, 1) = str2double(row{5});
end
end

function aggregate = makeExternalSummary(numTrials, trialSeeds)
numSensors = 8;
aggregate.armNames = {'Zheng-style subdensity GA-LMB adaptation', ...
    'Gao-style local-trust GA-LMB adaptation'};
aggregate.commConfig = makeCommConfig();
aggregate.arms = makeExternalArms();
aggregate.trialSeeds = reshape(trialSeeds, 1, []);
aggregate.pDropBySensorTrials = zeros(numTrials, numSensors);
aggregate.consensusTrials.ospa = repmat([2.1, 2.2], numTrials, 1);
aggregate.consensusTrials.gospa = repmat([3.6, 3.7], numTrials, 1);
aggregate.consensusTrials.pos = repmat([1.9, 2.0], numTrials, 1);
aggregate.consensusTrials.card = repmat([0.4, 0.5], numTrials, 1);
aggregate.localTrials.eOspa = repeatLocal([2.5, 2.6], numTrials, numSensors);
aggregate.localTrials.hOspa = zeros(numTrials, numSensors, 2);
aggregate.localTrials.rmse = repeatLocal([1.6, 1.7], numTrials, numSensors);
aggregate.localTrials.cardErr = repeatLocal([0.8, 0.9], numTrials, numSensors);
aggregate.runtime.filterSeconds = repmat([65, 70], numTrials, 1);
end

function values = repeatLocal(perArmValues, numTrials, numSensors)
values = repmat(reshape(perArmValues, 1, 1, []), numTrials, numSensors, 1);
end

function commConfig = makeCommConfig()
commConfig = struct( ...
    'level', 2, ...
    'globalMaxMeasurementsPerStep', 80, ...
    'sensorWeights', ones(1, 8) / 8, ...
    'priorityPolicy', 'weightedPriority', ...
    'measurementSelectionPolicy', 'random', ...
    'linkModel', 'fixed', ...
    'pDrop', 0.2, ...
    'pDropLevels', [0, 0.1, 0.2, 0.5], ...
    'pDropLevelCounts', [1, 4, 1, 2], ...
    'maxOutageNodes', 1);
end

function arms = makeCoreArms()
arms = repmat(struct('name', '', 'adaptiveFusion', struct()), 1, 4);
arms(1).name = 'fixed weights';
arms(1).adaptiveFusion = struct('enabled', false, 'method', 'factorized');
arms(2).name = 'Cao-Zhao FID-FIA baseline';
arms(2).adaptiveFusion = struct( ...
    'enabled', true, 'method', 'fidFia', ...
    'fidFiaUseEma', false, 'fidFiaMinWeight', 0);
arms(3).name = '+structure-aware decoupled KLA';
arms(3).adaptiveFusion = struct( ...
    'enabled', true, 'method', 'factorized', ...
    'useDecoupledKla', true, 'useStructureAwareKla', true, ...
    'useCovariance', true, 'useLinkQuality', true, ...
    'useExistenceConfidence', true, 'useFidFiaExistence', false);
arms(4).name = '+FID-FIA existence refinement';
arms(4).adaptiveFusion = struct( ...
    'enabled', true, 'method', 'factorized', ...
    'useFidFiaExistence', true, 'fidFiaExistenceStrength', 4, ...
    'fidFiaExistenceMinScore', 0, 'existenceEmaAlpha', 0, ...
    'existenceMinWeight', 0);
end

function arms = makePdArms()
arms = struct('name', 'PD-weighted GA', ...
    'adaptiveFusion', struct('enabled', true, 'method', 'pdWeightedGa'));
end

function arms = makeExternalArms()
arms = repmat(struct('name', '', 'adaptiveFusion', struct()), 1, 2);
arms(1).name = 'Zheng-style subdensity GA-LMB adaptation';
arms(1).adaptiveFusion = struct( ...
    'enabled', true, 'method', 'zhengStyleLmb', ...
    'zhengSpatialPrecisionPower', 1, ...
    'zhengExistenceMarginPower', 1, ...
    'zhengExistenceMinScore', 0.05);
arms(2).name = 'Gao-style local-trust GA-LMB adaptation';
arms(2).adaptiveFusion = struct( ...
    'enabled', true, 'method', 'gaoStyleResilient', ...
    'gaoDistanceScale', 2, 'gaoMinCredibility', 0.05, ...
    'gaoSpatialDistancePower', 1, 'gaoExistenceDistancePower', 1, ...
    'gaoHardReject', false);
end

function assertArtifacts(outputDir, names)
for idx = 1:numel(names)
    path = fullfile(outputDir, names{idx});
    assert(exist(path, 'file') == 2, 'Missing generated artifact: %s', path);
end
end
