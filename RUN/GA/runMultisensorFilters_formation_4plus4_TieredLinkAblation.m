function [reportPath, summary] = runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
    numberOfTrials, baseSeed, useFixedSeed, commConfigOverrides, writeReport, finalArmMode, adaptiveFusionOverrides, armSelection)
% RUNMULTISENSORFILTERS_FORMATION_4PLUS4_TIEREDLINKABLATION
% Ablation under tiered packet-drop configuration:
%   fixed weights -> +covariance -> +link quality -> +(robust NIS or freshness)

close all; clc;
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end
projectRoot = resolveProjectRoot(scriptDir);
addpath(projectRoot);
setPath;

if nargin < 1 || isempty(numberOfTrials)
    numberOfTrials = 1;
end
if nargin < 2 || isempty(baseSeed)
    baseSeed = 1;
end
if nargin < 3 || isempty(useFixedSeed)
    useFixedSeed = true;
end
if nargin < 4 || isempty(commConfigOverrides)
    commConfigOverrides = struct();
end
if nargin < 5 || isempty(writeReport)
    writeReport = true;
end
if nargin < 6 || isempty(finalArmMode)
    finalArmMode = 'robustNIS';
end
if nargin < 7 || isempty(adaptiveFusionOverrides)
    adaptiveFusionOverrides = struct();
end
if nargin < 8
    armSelection = [];
end

reportPath = '';
summary = struct();

staggeredBirths = true;
leaderSensor = 8;
sensorCommRange = 150;
fusionWeighting = 'Metropolis';
baseAdaptiveFusionConfig = struct( ...
    'enabled', true, ...
    'emaAlpha', 0.7, ...
    'minWeight', 0.05, ...
    'useCovariance', true, ...
    'useLinkQuality', true, ...
    'useCardinalityConsensus', false, ...
    'cardinalityConsensusScale', 4.0, ...
    'cardinalityConsensusMinScore', 0.4, ...
    'useExistenceConfidence', false, ...
    'existenceConfidenceMinScore', 0.6, ...
    'existenceConfidencePower', 1.0, ...
    'useDecoupledKla', false, ...
    'spatialEmaAlpha', 0.7, ...
    'existenceEmaAlpha', 0.7, ...
    'spatialMinWeight', 0.05, ...
    'existenceMinWeight', 0.05, ...
    'spatialCovariancePower', 1.0, ...
    'spatialLinkQualityPower', 1.0, ...
    'existenceLinkQualityPower', 1.0, ...
    'existenceConfidenceWeightPower', 1.0, ...
    'spatialDecouplingStrength', 1.0, ...
    'existenceDecouplingStrength', 1.0, ...
    'useStructureAwareKla', false, ...
    'usePosteriorStructureConsistency', true, ...
    'spatialStructureStrength', 0.0, ...
    'existenceStructureStrength', 0.0, ...
    'structureReliabilityPower', 0.0, ...
    'structureReliabilityMinScore', 0.25, ...
    'useFreshness', false, ...
    'freshnessDecay', 0.5, ...
    'freshnessMinScore', 0.4, ...
    'useHistory', false, ...
    'historyEmaAlpha', 0.8, ...
    'historyScale', 2.0, ...
    'historyMinScore', 0.4, ...
    'historyCovWeight', 0.4, ...
    'historyInnovationWeight', 0.4, ...
    'historyCardinalityWeight', 0.2, ...
    'nisQuantileEnabled', true, ...
    'nisQuantile', 0.7, ...
    'nisConsistencyConfidence', 0.7, ...
    'nisPenaltyScale', 4.0, ...
    'nisPenaltyMin', 0.3, ...
    'nisPenaltyLowerScale', 1.0, ...
    'nisPenaltyUpperScale', 6.0, ...
    'nisPenaltyLowerPower', 2.0, ...
    'nisPenaltyUpperPower', 2.0, ...
    'nisEmaEnabled', true, ...
    'nisEmaAlpha', 0.7, ...
    'useNIS', true, ...
    'robustNIS', true, ...
    'robustNISMin', 0.3);
baseAdaptiveFusionConfig = mergeStructFields(baseAdaptiveFusionConfig, adaptiveFusionOverrides);

numberOfSensors = 8;
clutterRates = 3 * ones(1, numberOfSensors);
detectionProbabilities = 0.9 * ones(1, numberOfSensors);
q = 3 * ones(1, numberOfSensors);

commConfig = struct();
commConfig.level = 2;
commConfig.globalMaxMeasurementsPerStep = 80;
commConfig.sensorWeights = ones(1, numberOfSensors) / numberOfSensors;
commConfig.priorityPolicy = 'weightedPriority';
commConfig.measurementSelectionPolicy = 'random';
commConfig.linkModel = 'fixed';
commConfig.pDrop = 0.2;
commConfig.pDropLevels = [0, 0.1, 0.2, 0.5];
commConfig.pDropLevelCounts = [1, 4, 1, 2];
commConfig.maxOutageNodes = 1;
commConfig = mergeStructFields(commConfig, commConfigOverrides);

sensorMotionConfig = struct();
sensorMotionConfig.enabled = true;
sensorMotionConfig.motionType = 'CV';
sensorMotionConfig.processNoiseStd = 0.0;
sensorMotionConfig.initialStates = buildSensorInitialStates();

targetFormationConfig = struct();
targetFormationConfig.targetFormationEnabled = true;
targetFormationConfig.targetFormationStaggeredBirths = staggeredBirths;
targetFormationConfig.targetFormationBirthInterval = 8;
targetFormationConfig.targetFormationStartTime = 1;
targetFormationConfig.targetFormationLifeSpan = 100;
targetFormationConfig.targetBirthStates = buildTargetBirthStates();
targetFormationConfig.targetFormationCount = size(targetFormationConfig.targetBirthStates, 2);

arms = buildArms(baseAdaptiveFusionConfig, finalArmMode);
arms = selectArms(arms, armSelection);
armNames = {arms.name};
numArms = numel(arms);

eOspa = zeros(numberOfTrials, numberOfSensors, numArms);
hOspa = zeros(numberOfTrials, numberOfSensors, numArms);
rmse = zeros(numberOfTrials, numberOfSensors, numArms);
cardErr = zeros(numberOfTrials, numberOfSensors, numArms);
consOspa = zeros(numberOfTrials, numArms);
consPos = zeros(numberOfTrials, numArms);
consCard = zeros(numberOfTrials, numArms);
consOspaSeries = [];
consPosSeries = [];
consCardSeries = [];
pDropBySensorTrials = zeros(numberOfTrials, numberOfSensors);

for trial = 1:numberOfTrials
    fprintf('Trial %d/%d\n', trial, numberOfTrials);
    if useFixedSeed
        rng(baseSeed + trial);
    end

    model = generateMultisensorModel(numberOfSensors, clutterRates, ...
        detectionProbabilities, q, 'GA', 'LBP', 'Formation', ...
        sensorMotionConfig, targetFormationConfig);
    model.sensorCommRange = sensorCommRange;
    model.fusionWeighting = fusionWeighting;
    model.sensorFovEnabled = true;
    model.sensorFovHalfAngleDeg = 60;
    model.sensorFovRange = 60000;

    [~, measurements, groundTruthRfs, sensorTrajectories] = generateMultisensorGroundTruth(model);
    model.sensorTrajectories = sensorTrajectories;

    [measurementsDelivered, commStats] = applyCommunicationModel(measurements, model, commConfig);
    pDropBySensorTrials(trial, :) = reshape(commStats.pDropBySensor, 1, []);

    for armIdx = 1:numArms
        fprintf('  Arm %d/%d: %s\n', armIdx, numArms, arms(armIdx).name);
        armModel = model;
        armModel.adaptiveFusion = arms(armIdx).adaptiveFusion;
        neighborMap = buildNeighborMap4Plus4(numberOfSensors);
        [stateEstimatesBySensor, localModels] = runDistributedLmbFilter( ...
            armModel, measurementsDelivered, sensorTrajectories, neighborMap, commStats);

        for s = 1:numberOfSensors
            [eArm, hArm, cardArm] = computeSimulationOspa(localModels{s}, groundTruthRfs, stateEstimatesBySensor{s});
            eOspa(trial, s, armIdx) = mean(eArm);
            hOspa(trial, s, armIdx) = mean(hArm);
            rmse(trial, s, armIdx) = mean(computeSetRmseOverTime(stateEstimatesBySensor{s}, groundTruthRfs), 'omitnan');
            cardErr(trial, s, armIdx) = mean(abs(cardArm - groundTruthRfs.cardinality));
        end

        [posArm, cardArm, ospaArm] = computeConsensusMetrics(stateEstimatesBySensor, armModel);
        consOspa(trial, armIdx) = mean(ospaArm);
        consPos(trial, armIdx) = mean(posArm, 'omitnan');
        consCard(trial, armIdx) = mean(cardArm);
        if isempty(consOspaSeries)
            simLength = numel(ospaArm);
            consOspaSeries = zeros(numberOfTrials, simLength, numArms);
            consPosSeries = NaN(numberOfTrials, simLength, numArms);
            consCardSeries = zeros(numberOfTrials, simLength, numArms);
        end
        consOspaSeries(trial, :, armIdx) = reshape(ospaArm, 1, []);
        consPosSeries(trial, :, armIdx) = reshape(posArm, 1, []);
        consCardSeries(trial, :, armIdx) = reshape(cardArm, 1, []);
    end
end

fprintf('=====================================\n');
fprintf('GA Tiered Link Ablation (N=%d)\n', numberOfTrials);
fprintf('Order: %s\n', strjoin(armNames, ' -> '));
fprintf('Tier levels=%s, counts=%s\n', mat2str(commConfig.pDropLevels, 3), mat2str(commConfig.pDropLevelCounts));
fprintf('pDropBySensor=%s\n', mat2str(mean(pDropBySensorTrials, 1), 4));
fprintf('=====================================\n');
for armIdx = 1:numArms
    fprintf('%s: OSPA %.6f, RMSE %.6f, Card %.6f\n', arms(armIdx).name, ...
        mean(consOspa(:, armIdx)), mean(consPos(:, armIdx), 'omitnan'), mean(consCard(:, armIdx)));
end

summary.armNames = armNames;
summary.consensus.ospa = mean(consOspa, 1);
summary.consensus.pos = mean(consPos, 1, 'omitnan');
summary.consensus.card = mean(consCard, 1);
summary.consensusTrials.ospa = consOspa;
summary.consensusTrials.pos = consPos;
summary.consensusTrials.card = consCard;
summary.local.eOspa = squeeze(mean(eOspa, 1));
summary.local.hOspa = squeeze(mean(hOspa, 1));
summary.local.rmse = squeeze(mean(rmse, 1));
summary.local.cardErr = squeeze(mean(cardErr, 1));
summary.local.meanAcrossSensors.eOspa = computePerArmGlobalMeans(eOspa, false);
summary.local.meanAcrossSensors.hOspa = computePerArmGlobalMeans(hOspa, false);
summary.local.meanAcrossSensors.rmse = computePerArmGlobalMeans(rmse, true);
summary.local.meanAcrossSensors.cardErr = computePerArmGlobalMeans(cardErr, false);
summary.localTrials.eOspa = eOspa;
summary.localTrials.hOspa = hOspa;
summary.localTrials.rmse = rmse;
summary.localTrials.cardErr = cardErr;
summary.trialSeeds = computeTrialSeeds(numberOfTrials, baseSeed, useFixedSeed);
if ~isempty(consOspaSeries)
    summary.consensusSeries.time = (1:size(consOspaSeries, 2))';
    summary.consensusSeries.ospa = squeeze(mean(consOspaSeries, 1));
    summary.consensusSeries.pos = squeeze(mean(consPosSeries, 1, 'omitnan'));
    summary.consensusSeries.card = squeeze(mean(consCardSeries, 1));
    summary.consensusSeries.armNames = armNames;
end
summary.pDropBySensorTrials = pDropBySensorTrials;
summary.meanPDropBySensor = mean(pDropBySensorTrials, 1);
summary.commConfig = commConfig;
summary.arms = arms;

if writeReport
    reportDir = fullfile(projectRoot, 'RUN', 'GA');
    if ~exist(reportDir, 'dir')
        mkdir(reportDir);
    end
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    reportPrefix = '';
    if any(strcmpi(finalArmMode, {'freshness', 'fresh', 'cardinality', 'cardinalityconsensus', 'cardinality_consensus'}))
        reportPrefix = 'Del_';
    end
    reportName = sprintf('%sGA_TIERED_LINK_ABLATION_N%d_SEED%d_%s.md', ...
        reportPrefix, numberOfTrials, baseSeed, timestamp);
    reportPath = fullfile(reportDir, reportName);
    writeAblationReport(reportPath, numberOfTrials, baseSeed, useFixedSeed, ...
        sensorCommRange, fusionWeighting, leaderSensor, commConfig, pDropBySensorTrials, ...
        arms, consOspa, consPos, consCard, eOspa, hOspa, rmse, cardErr, finalArmMode);
    fprintf('Report written: %s\n', reportPath);
end
end

function arms = selectArms(arms, armSelection)
if nargin < 2 || isempty(armSelection)
    return;
end

if isnumeric(armSelection)
    armIdx = unique(max(1, min(numel(arms), round(armSelection(:)'))));
    arms = arms(armIdx);
    return;
end

if ischar(armSelection) || isstring(armSelection)
    requested = cellstr(armSelection);
elseif iscell(armSelection)
    requested = cellfun(@char, armSelection, 'UniformOutput', false);
else
    return;
end

matched = false(1, numel(arms));
for i = 1:numel(requested)
    query = lower(strtrim(requested{i}));
    if isempty(query)
        continue;
    end
    for armIdx = 1:numel(arms)
        if strcmpi(query, 'final') && armIdx == numel(arms)
            matched(armIdx) = true;
        elseif contains(lower(arms(armIdx).name), query)
            matched(armIdx) = true;
        end
    end
end

if any(matched)
    arms = arms(matched);
end
end

function arms = buildArms(baseAdaptiveFusionConfig, finalArmMode)
arms = repmat(struct('name', '', 'adaptiveFusion', struct()), 1, 5);

cfg = baseAdaptiveFusionConfig;
cfg.enabled = false;
cfg.useDecoupledKla = false;
cfg.useCovariance = false;
cfg.useLinkQuality = false;
cfg.useNIS = false;
arms(1).name = 'fixed weights';
arms(1).adaptiveFusion = cfg;

cfg = baseAdaptiveFusionConfig;
cfg.enabled = true;
cfg.useDecoupledKla = false;
cfg.useCovariance = true;
cfg.useLinkQuality = false;
cfg.useNIS = false;
arms(2).name = '+covariance';
arms(2).adaptiveFusion = cfg;

cfg = baseAdaptiveFusionConfig;
cfg.enabled = true;
cfg.useDecoupledKla = false;
cfg.useCovariance = true;
cfg.useLinkQuality = true;
cfg.useNIS = false;
arms(3).name = '+link quality';
arms(3).adaptiveFusion = cfg;

switch lower(finalArmMode)
    case {'fidfia', 'fid_fia', 'fidfiabaseline', 'fid_fia_baseline', ...
            'fisherfia', 'fisher_fia', 'informationgeometry', ...
            'information_geometry', 'caozhao', 'cao_zhao'}
        arms = repmat(struct('name', '', 'adaptiveFusion', struct()), 1, 3);

        cfg = baseAdaptiveFusionConfig;
        cfg.enabled = false;
        cfg.method = 'factorized';
        cfg.useDecoupledKla = false;
        cfg.useCovariance = false;
        cfg.useLinkQuality = false;
        cfg.useNIS = false;
        arms(1).name = 'fixed weights';
        arms(1).adaptiveFusion = cfg;

        cfg = baseAdaptiveFusionConfig;
        cfg.enabled = true;
        cfg.method = 'fidFia';
        cfg.useDecoupledKla = false;
        cfg.useStructureAwareKla = false;
        cfg.useCovariance = false;
        cfg.useLinkQuality = false;
        cfg.useExistenceConfidence = false;
        cfg.useFreshness = false;
        cfg.useHistory = false;
        cfg.useNIS = false;
        cfg.fidFiaUseEma = false;
        cfg.fidFiaMinWeight = 0.0;
        cfg.fidFiaUseExistenceWeight = true;
        cfg.fidFiaExistencePower = 1.0;
        cfg.fidFiaQuadraturePoints = 3;
        cfg.fidFiaUseDetectionProbability = true;
        arms(2).name = 'Cao-Zhao FID-FIA baseline';
        arms(2).adaptiveFusion = cfg;

        cfg = baseAdaptiveFusionConfig;
        cfg.enabled = true;
        cfg.method = 'factorized';
        cfg.useDecoupledKla = false;
        cfg.useCovariance = true;
        cfg.useLinkQuality = true;
        cfg.useFreshness = false;
        cfg.useNIS = false;
        cfg.useExistenceConfidence = true;
        cfg.useDecoupledKla = true;
        cfg.useStructureAwareKla = true;
        cfg.usePosteriorStructureConsistency = false;
        if abs(cfg.existenceConfidenceMinScore - 0.6) < 1e-9
            cfg.existenceConfidenceMinScore = 0.85;
        end
        if abs(cfg.existenceConfidencePower - 1.0) < 1e-9
            cfg.existenceConfidencePower = 2.0;
        end
        if abs(cfg.spatialDecouplingStrength - 1.0) < 1e-9
            cfg.spatialDecouplingStrength = 0.5;
        end
        if abs(cfg.existenceDecouplingStrength - 1.0) < 1e-9
            cfg.existenceDecouplingStrength = 0.15;
        end
        if cfg.spatialStructureStrength <= 0
            cfg.spatialStructureStrength = 0.45;
        end
        if cfg.existenceStructureStrength <= 0
            cfg.existenceStructureStrength = 0.08;
        end
        if cfg.structureReliabilityPower <= 0
            cfg.structureReliabilityPower = 0.30;
        end
        arms(3).name = '+structure-aware decoupled KLA';
        arms(3).adaptiveFusion = cfg;
    case {'freshness', 'fresh'}
        cfg = baseAdaptiveFusionConfig;
        cfg.enabled = true;
        cfg.useDecoupledKla = false;
        cfg.useCovariance = true;
        cfg.useLinkQuality = true;
        cfg.useFreshness = false;
        cfg.useNIS = false;
        cfg.useFreshness = true;
        arms(4).name = '+freshness';
        arms(4).adaptiveFusion = cfg;
        arms = arms(1:4);
    case {'cardinality', 'cardinalityconsensus', 'cardinality_consensus'}
        cfg = baseAdaptiveFusionConfig;
        cfg.enabled = true;
        cfg.useDecoupledKla = false;
        cfg.useCovariance = true;
        cfg.useLinkQuality = true;
        cfg.useFreshness = false;
        cfg.useNIS = false;
        cfg.useCardinalityConsensus = true;
        arms(4).name = '+cardinality consensus';
        arms(4).adaptiveFusion = cfg;
        arms = arms(1:4);
    case {'existence', 'existenceconfidence', 'existence_confidence'}
        cfg = baseAdaptiveFusionConfig;
        cfg.enabled = true;
        cfg.useDecoupledKla = false;
        cfg.useCovariance = true;
        cfg.useLinkQuality = true;
        cfg.useFreshness = false;
        cfg.useNIS = false;
        cfg.useExistenceConfidence = true;
        arms(4).name = '+existence confidence';
        arms(4).adaptiveFusion = cfg;
        arms = arms(1:4);
    case {'decoupled', 'decoupledkla', 'decoupled_kla'}
        cfg = baseAdaptiveFusionConfig;
        cfg.enabled = true;
        cfg.useDecoupledKla = false;
        cfg.useCovariance = true;
        cfg.useLinkQuality = true;
        cfg.useFreshness = false;
        cfg.useNIS = false;
        cfg.useExistenceConfidence = true;
        arms(4).name = '+existence confidence';
        arms(4).adaptiveFusion = cfg;

        cfg = baseAdaptiveFusionConfig;
        cfg.enabled = true;
        cfg.useDecoupledKla = false;
        cfg.useCovariance = true;
        cfg.useLinkQuality = true;
        cfg.useFreshness = false;
        cfg.useNIS = false;
        cfg.useExistenceConfidence = true;
        cfg.useDecoupledKla = true;
        arms(5).name = '+decoupled KLA';
        arms(5).adaptiveFusion = cfg;
    case {'structureaware', 'structure_aware', 'structure-aware', ...
            'structureawaredecoupledkla', 'structure_aware_decoupled_kla', ...
            'structure-aware-decoupled-kla'}
        cfg = baseAdaptiveFusionConfig;
        cfg.enabled = true;
        cfg.useDecoupledKla = false;
        cfg.useCovariance = true;
        cfg.useLinkQuality = true;
        cfg.useFreshness = false;
        cfg.useNIS = false;
        cfg.useExistenceConfidence = true;
        arms(4).name = '+existence confidence';
        arms(4).adaptiveFusion = cfg;

        cfg = baseAdaptiveFusionConfig;
        cfg.enabled = true;
        cfg.useDecoupledKla = false;
        cfg.useCovariance = true;
        cfg.useLinkQuality = true;
        cfg.useFreshness = false;
        cfg.useNIS = false;
        cfg.useExistenceConfidence = true;
        cfg.useDecoupledKla = true;
        cfg.useStructureAwareKla = true;
        cfg.usePosteriorStructureConsistency = false;
        if abs(cfg.existenceConfidenceMinScore - 0.6) < 1e-9
            cfg.existenceConfidenceMinScore = 0.85;
        end
        if abs(cfg.existenceConfidencePower - 1.0) < 1e-9
            cfg.existenceConfidencePower = 2.0;
        end
        if abs(cfg.spatialDecouplingStrength - 1.0) < 1e-9
            cfg.spatialDecouplingStrength = 0.5;
        end
        if abs(cfg.existenceDecouplingStrength - 1.0) < 1e-9
            cfg.existenceDecouplingStrength = 0.15;
        end
        if cfg.spatialStructureStrength <= 0
            cfg.spatialStructureStrength = 0.45;
        end
        if cfg.existenceStructureStrength <= 0
            cfg.existenceStructureStrength = 0.08;
        end
        if cfg.structureReliabilityPower <= 0
            cfg.structureReliabilityPower = 0.30;
        end
        arms(5).name = '+structure-aware decoupled KLA';
        arms(5).adaptiveFusion = cfg;
    otherwise
        cfg = baseAdaptiveFusionConfig;
        cfg.enabled = true;
        cfg.useDecoupledKla = false;
        cfg.useCovariance = true;
        cfg.useLinkQuality = true;
        cfg.useFreshness = false;
        cfg.useNIS = true;
        cfg.robustNIS = true;
        arms(4).name = '+robust NIS';
        arms(4).adaptiveFusion = cfg;
        arms = arms(1:4);
end
end

function writeAblationReport(reportPath, numberOfTrials, baseSeed, useFixedSeed, ...
    sensorCommRange, fusionWeighting, leaderSensor, commConfig, pDropBySensorTrials, ...
    arms, consOspa, consPos, consCard, eOspa, hOspa, rmse, cardErr, finalArmMode)

fid = fopen(reportPath, 'w');
if fid < 0
    warning('Unable to write report: %s', reportPath);
    return;
end

timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');
fprintf(fid, '# GA Tiered Link Ablation (%s)\n\n', timestamp);
fprintf(fid, 'Comparison order: %s\n\n', strjoin({arms.name}, ' -> '));
fprintf(fid, '## Run Config\n');
fprintf(fid, '- Trials: %d\n', numberOfTrials);
fprintf(fid, '- baseSeed: %d (fixed=%d)\n', baseSeed, useFixedSeed);
if useFixedSeed
    fprintf(fid, '- trialSeeds: %s\n', mat2str(computeTrialSeeds(numberOfTrials, baseSeed, useFixedSeed)));
end
fprintf(fid, '- sensorCommRange: %d\n', sensorCommRange);
fprintf(fid, '- fusionWeighting: %s\n', fusionWeighting);
fprintf(fid, '- leaderSensor: %d\n', leaderSensor);
fprintf(fid, '- linkModel: %s\n', getField(commConfig, 'linkModel', 'fixed'));
fprintf(fid, '- pDrop target mean: %.3f\n', getField(commConfig, 'pDrop', 0));
fprintf(fid, '- pDropLevels: %s\n', mat2str(getField(commConfig, 'pDropLevels', []), 3));
fprintf(fid, '- pDropLevelCounts: %s\n\n', mat2str(getField(commConfig, 'pDropLevelCounts', [])));
fprintf(fid, '- finalArmMode: %s\n\n', finalArmMode);

fprintf(fid, '## Arm Configs\n');
for armIdx = 1:numel(arms)
    cfg = arms(armIdx).adaptiveFusion;
    fprintf(fid, '### %s\n', arms(armIdx).name);
    fprintf(fid, '- enabled: %d\n', getField(cfg, 'enabled', false));
    fprintf(fid, '- method: %s\n', char(getField(cfg, 'method', 'factorized')));
    fprintf(fid, '- useCovariance: %d\n', getField(cfg, 'useCovariance', false));
    fprintf(fid, '- useLinkQuality: %d\n', getField(cfg, 'useLinkQuality', false));
    fprintf(fid, '- useExistenceConfidence: %d\n', getField(cfg, 'useExistenceConfidence', false));
    fprintf(fid, '- useNIS: %d\n', getField(cfg, 'useNIS', false));
    fprintf(fid, '- useDecoupledKla: %d\n', getField(cfg, 'useDecoupledKla', false));
    fprintf(fid, '- useStructureAwareKla: %d\n', getField(cfg, 'useStructureAwareKla', false));
    fprintf(fid, '- usePosteriorStructureConsistency: %d\n', getField(cfg, 'usePosteriorStructureConsistency', false));
    fprintf(fid, '- existenceConfidenceMinScore: %.3f\n', getField(cfg, 'existenceConfidenceMinScore', 0));
    fprintf(fid, '- existenceConfidencePower: %.3f\n', getField(cfg, 'existenceConfidencePower', 0));
    fprintf(fid, '- spatialDecouplingStrength: %.3f\n', getField(cfg, 'spatialDecouplingStrength', 0));
    fprintf(fid, '- existenceDecouplingStrength: %.3f\n', getField(cfg, 'existenceDecouplingStrength', 0));
    fprintf(fid, '- spatialStructureStrength: %.3f\n', getField(cfg, 'spatialStructureStrength', 0));
    fprintf(fid, '- existenceStructureStrength: %.3f\n', getField(cfg, 'existenceStructureStrength', 0));
    fprintf(fid, '- structureReliabilityPower: %.3f\n', getField(cfg, 'structureReliabilityPower', 0));
    fprintf(fid, '- structureReliabilityMinScore: %.3f\n', getField(cfg, 'structureReliabilityMinScore', 0));
    if strcmpi(getField(cfg, 'method', 'factorized'), 'fidFia')
        fprintf(fid, '- fidFiaUseExistenceWeight: %d\n', getField(cfg, 'fidFiaUseExistenceWeight', true));
        fprintf(fid, '- fidFiaExistencePower: %.3f\n', getField(cfg, 'fidFiaExistencePower', 1.0));
        fprintf(fid, '- fidFiaQuadraturePoints: %d\n', getField(cfg, 'fidFiaQuadraturePoints', 3));
        fprintf(fid, '- fidFiaUseDetectionProbability: %d\n', getField(cfg, 'fidFiaUseDetectionProbability', true));
        fprintf(fid, '- fidFiaUseEma: %d\n', getField(cfg, 'fidFiaUseEma', false));
        fprintf(fid, '- fidFiaMinWeight: %.3f\n\n', getField(cfg, 'fidFiaMinWeight', 0));
    else
        fprintf(fid, '\n');
    end
end

fprintf(fid, '## Per-Trial pDropBySensor\n');
for trial = 1:size(pDropBySensorTrials, 1)
    fprintf(fid, '- Trial %d: %s\n', trial, mat2str(pDropBySensorTrials(trial, :), 4));
end
fprintf(fid, '\n');

fprintf(fid, '## Per-Trial Consensus Metrics\n');
fprintf(fid, '| Trial | Seed | Arm | OSPA | RMSE | Cardinality |\n');
fprintf(fid, '|------:|-----:|:----|-----:|-----:|------------:|\n');
trialSeeds = computeTrialSeeds(numberOfTrials, baseSeed, useFixedSeed);
for trial = 1:numberOfTrials
    for armIdx = 1:numel(arms)
        fprintf(fid, '| %d | %.0f | %s | %.6f | %.6f | %.6f |\n', ...
            trial, trialSeeds(trial), arms(armIdx).name, ...
            consOspa(trial, armIdx), consPos(trial, armIdx), consCard(trial, armIdx));
    end
end
fprintf(fid, '\n');

fprintf(fid, '## Consensus Metrics (mean across trials)\n');
fprintf(fid, '| Arm | OSPA | RMSE | Cardinality |\n');
fprintf(fid, '|:----|-----:|-----:|------------:|\n');
for armIdx = 1:numel(arms)
    fprintf(fid, '| %s | %.6f | %.6f | %.6f |\n', arms(armIdx).name, ...
        mean(consOspa(:, armIdx)), mean(consPos(:, armIdx), 'omitnan'), mean(consCard(:, armIdx)));
end

fprintf(fid, '\n## Consensus Metrics With Trial Variability\n');
writeMetricStatsTable(fid, {arms.name}, {'OSPA', 'RMSE', 'Cardinality'}, ...
    {consOspa, consPos, consCard}, [false, true, false]);

if numel(arms) >= 2
    fprintf(fid, '\n## Paired Improvements Relative to Fixed Weights\n');
    writePairedImprovementTable(fid, {arms.name}, {'OSPA', 'RMSE', 'Cardinality'}, ...
        {consOspa, consPos, consCard}, [false, true, false]);
end

fprintf(fid, '\n## Local Tracking Metrics (mean across sensors and trials)\n');
fprintf(fid, '| Arm | E-OSPA | H-OSPA | RMSE | CardErr |\n');
fprintf(fid, '|:----|-------:|-------:|-----:|--------:|\n');
for armIdx = 1:numel(arms)
    eOspaMean = computeGlobalMean(eOspa(:, :, armIdx), false);
    hOspaMean = computeGlobalMean(hOspa(:, :, armIdx), false);
    rmseMean = computeGlobalMean(rmse(:, :, armIdx), true);
    cardErrMean = computeGlobalMean(cardErr(:, :, armIdx), false);
    fprintf(fid, '| %s | %.6f | %.6f | %.6f | %.6f |\n', arms(armIdx).name, ...
        eOspaMean, hOspaMean, rmseMean, cardErrMean);
end

fprintf(fid, '\n## Local Tracking Metrics With Trial Variability\n');
localEOspaTrial = computeTrialSensorMeans(eOspa, false);
localHOspaTrial = computeTrialSensorMeans(hOspa, false);
localRmseTrial = computeTrialSensorMeans(rmse, true);
localCardTrial = computeTrialSensorMeans(cardErr, false);
writeMetricStatsTable(fid, {arms.name}, {'E-OSPA', 'H-OSPA', 'RMSE', 'CardErr'}, ...
    {localEOspaTrial, localHOspaTrial, localRmseTrial, localCardTrial}, [false, false, true, false]);

if numel(arms) >= 2
    fprintf(fid, '\n## Paired Local-Metric Improvements Relative to Fixed Weights\n');
    writePairedImprovementTable(fid, {arms.name}, {'E-OSPA', 'H-OSPA', 'RMSE', 'CardErr'}, ...
        {localEOspaTrial, localHOspaTrial, localRmseTrial, localCardTrial}, [false, false, true, false]);
end

fprintf(fid, '\n## Local Tracking Metrics By Sensor (mean across trials)\n');
fprintf(fid, '| Sensor | Arm | E-OSPA | H-OSPA | RMSE | CardErr |\n');
fprintf(fid, '|------:|:----|-------:|-------:|-----:|--------:|\n');
for sensorIdx = 1:size(eOspa, 2)
    for armIdx = 1:numel(arms)
        fprintf(fid, '| %d | %s | %.6f | %.6f | %.6f | %.6f |\n', sensorIdx, arms(armIdx).name, ...
            mean(eOspa(:, sensorIdx, armIdx)), mean(hOspa(:, sensorIdx, armIdx)), ...
            mean(rmse(:, sensorIdx, armIdx), 'omitnan'), mean(cardErr(:, sensorIdx, armIdx)));
    end
end

fclose(fid);
end

function seeds = computeTrialSeeds(numberOfTrials, baseSeed, useFixedSeed)
if useFixedSeed
    seeds = baseSeed + (1:numberOfTrials);
else
    seeds = NaN(1, numberOfTrials);
end
end

function writeMetricStatsTable(fid, armNames, metricNames, metricArrays, omitnanFlags)
fprintf(fid, '| Arm | Metric | Mean +/- Std | 95%% CI | N |\n');
fprintf(fid, '|:----|:-------|-------------:|:-------|--:|\n');
for metricIdx = 1:numel(metricNames)
    data = metricArrays{metricIdx};
    omitnanFlag = omitnanFlags(metricIdx);
    for armIdx = 1:numel(armNames)
        stats = summarizeVector(data(:, armIdx), omitnanFlag);
        fprintf(fid, '| %s | %s | %.6f +/- %.6f | [%.6f, %.6f] | %d |\n', ...
            armNames{armIdx}, metricNames{metricIdx}, stats.mean, stats.std, ...
            stats.ciLow, stats.ciHigh, stats.n);
    end
end
end

function writePairedImprovementTable(fid, armNames, metricNames, metricArrays, omitnanFlags)
fprintf(fid, '| Arm | Metric | Paired reduction | 95%% CI | Reduction | Wins | Sign-test p |\n');
fprintf(fid, '|:----|:-------|-----------------:|:-------|----------:|-----:|------------:|\n');
baselineIdx = 1;
for metricIdx = 1:numel(metricNames)
    data = metricArrays{metricIdx};
    omitnanFlag = omitnanFlags(metricIdx);
    baseline = data(:, baselineIdx);
    for armIdx = 2:numel(armNames)
        candidate = data(:, armIdx);
        valid = isfinite(baseline) & isfinite(candidate);
        if ~omitnanFlag
            valid = valid & ~isnan(baseline) & ~isnan(candidate);
        end
        deltas = baseline(valid) - candidate(valid);
        stats = summarizeVector(deltas, true);
        baseStats = summarizeVector(baseline(valid), true);
        if isfinite(baseStats.mean) && abs(baseStats.mean) > eps
            pct = 100 * stats.mean / baseStats.mean;
        else
            pct = NaN;
        end
        wins = sum(deltas > 0);
        pValue = signTestPvalue(deltas);
        fprintf(fid, '| %s | %s | %.6f +/- %.6f | [%.6f, %.6f] | %.2f%% | %d/%d | %.4g |\n', ...
            armNames{armIdx}, metricNames{metricIdx}, stats.mean, stats.std, ...
            stats.ciLow, stats.ciHigh, pct, wins, stats.n, pValue);
    end
end
end

function trialMeans = computeTrialSensorMeans(values, omitnanFlag)
numTrials = size(values, 1);
numArms = size(values, 3);
trialMeans = zeros(numTrials, numArms);
for trial = 1:numTrials
    for armIdx = 1:numArms
        sensorValues = reshape(values(trial, :, armIdx), [], 1);
        stats = summarizeVector(sensorValues, omitnanFlag);
        trialMeans(trial, armIdx) = stats.mean;
    end
end
end

function stats = summarizeVector(values, omitnanFlag)
values = values(:);
if nargin >= 2 && omitnanFlag
    values = values(isfinite(values));
end
stats.n = numel(values);
if stats.n == 0
    stats.mean = NaN;
    stats.std = NaN;
    stats.ciLow = NaN;
    stats.ciHigh = NaN;
    return;
end
stats.mean = mean(values);
if stats.n > 1
    stats.std = std(values, 0);
    halfWidth = tCritical95(stats.n - 1) * stats.std / sqrt(stats.n);
else
    stats.std = 0;
    halfWidth = 0;
end
stats.ciLow = stats.mean - halfWidth;
stats.ciHigh = stats.mean + halfWidth;
end

function tcrit = tCritical95(df)
table = [ ...
    12.706, 4.303, 3.182, 2.776, 2.571, ...
    2.447, 2.365, 2.306, 2.262, 2.228, ...
    2.201, 2.179, 2.160, 2.145, 2.131, ...
    2.120, 2.110, 2.101, 2.093, 2.086, ...
    2.080, 2.074, 2.069, 2.064, 2.060, ...
    2.056, 2.052, 2.048, 2.045, 2.042];
if df < 1
    tcrit = 0;
elseif df <= numel(table)
    tcrit = table(df);
else
    tcrit = 1.96;
end
end

function pValue = signTestPvalue(deltas)
deltas = deltas(:);
deltas = deltas(isfinite(deltas) & abs(deltas) > eps);
n = numel(deltas);
if n == 0
    pValue = NaN;
    return;
end
wins = sum(deltas > 0);
k = min(wins, n - wins);
tail = 0;
for i = 0:k
    logProb = gammaln(n + 1) - gammaln(i + 1) - gammaln(n - i + 1) - n * log(2);
    tail = tail + exp(logProb);
end
pValue = min(1, 2 * tail);
end

function values = computePerArmGlobalMeans(arrayLike, omitnanFlag)
if ndims(arrayLike) < 3
    values = computeGlobalMean(arrayLike, omitnanFlag);
    return;
end
numArms = size(arrayLike, 3);
values = zeros(1, numArms);
for armIdx = 1:numArms
    values(armIdx) = computeGlobalMean(arrayLike(:, :, armIdx), omitnanFlag);
end
end

function value = computeGlobalMean(arrayLike, omitnanFlag)
values = arrayLike(:);
if nargin >= 2 && omitnanFlag
    value = mean(values, 'omitnan');
else
    value = mean(values);
end
end

function merged = mergeStructFields(base, overrides)
merged = base;
if nargin < 2 || ~isstruct(overrides) || isempty(fieldnames(overrides))
    return;
end
fields = fieldnames(overrides);
for i = 1:numel(fields)
    merged.(fields{i}) = overrides.(fields{i});
end
end

function projectRoot = resolveProjectRoot(scriptDir)
if isempty(scriptDir)
    scriptDir = pwd;
end
projectRoot = scriptDir;
for k = 1:6
    if exist(fullfile(projectRoot, 'setPath.m'), 'file')
        return;
    end
    parent = fileparts(projectRoot);
    if isempty(parent) || strcmp(parent, projectRoot)
        break;
    end
    projectRoot = parent;
end
end

function sensorInitialStates = buildSensorInitialStates()
groupCenters = [-80, -80; 35, -35];
groupSpacing = 20;
formationType = 'Leader3';
formationSpeed = [0.8; 0];
sensorsPerGroup = 4;
numGroups = size(groupCenters, 2);
sensorInitialStates = cell(1, numGroups * sensorsPerGroup);
idx = 1;
for g = 1:numGroups
    offsets = localFormationOffsets(formationType, groupSpacing, sensorsPerGroup);
    center = groupCenters(:, g);
    for k = 1:sensorsPerGroup
        pos = center + offsets(:, k);
        sensorInitialStates{idx} = [pos; formationSpeed];
        idx = idx + 1;
    end
end
end

function neighborMap = buildNeighborMap4Plus4(numberOfSensors)
if numberOfSensors ~= 8
    error('buildNeighborMap4Plus4 expects numberOfSensors = 8.');
end
groupA = 1:4;
groupB = 5:8;
pairings = [1 5; 2 6; 3 7; 4 8];
neighborMap = cell(1, numberOfSensors);
for i = 1:4
    neighborMap{groupA(i)} = unique([groupA, pairings(i, 2)]);
    neighborMap{groupB(i)} = unique([groupB, pairings(i, 1)]);
end
end

function targetBirthStates = buildTargetBirthStates()
targetCenter = [0; 0];
groupCenters = [70, 80, 70; 80, 0, -80];
groupTypes = {'Triangle', 'Triangle', 'Leader3'};
groupCounts = [3, 3, 4];
groupSpacing = [30, 25, 20];
groupSpeed = [0.45, 0.45, 0.45];
totalTargets = sum(groupCounts);
targetBirthStates = zeros(4, totalTargets);
idx = 1;
for g = 1:numel(groupCounts)
    offsets = localFormationOffsets(groupTypes{g}, groupSpacing(g), groupCounts(g));
    center = groupCenters(:, g);
    dir = targetCenter - center;
    if norm(dir) < 1e-6
        dir = [-1; 0];
    end
    vel = (groupSpeed(g) / norm(dir)) * dir;
    for k = 1:groupCounts(g)
        pos = center + offsets(:, k);
        targetBirthStates(:, idx) = [pos; vel];
        idx = idx + 1;
    end
end
end

function [posConsensus, cardConsensus, ospaConsensus] = computeConsensusMetrics(stateEstimatesBySensor, model)
numSensors = numel(stateEstimatesBySensor);
simLength = numel(stateEstimatesBySensor{1}.mu);
posConsensus = zeros(1, simLength);
cardConsensus = zeros(1, simLength);
ospaConsensus = zeros(1, simLength);
for t = 1:simLength
    counts = zeros(1, numSensors);
    for s = 1:numSensors
        counts(s) = numel(stateEstimatesBySensor{s}.mu{t});
    end
    medCount = median(counts);
    cardConsensus(t) = mean(abs(counts - medCount));
    pairSum = 0;
    pairCount = 0;
    ospaSum = 0;
    ospaCount = 0;
    for i = 1:numSensors-1
        for j = i+1:numSensors
            d = estimateSetRmse(stateEstimatesBySensor{i}, stateEstimatesBySensor{j}, t);
            if ~isnan(d)
                pairSum = pairSum + d;
                pairCount = pairCount + 1;
            end
            dOspa = estimateSetOspaConsensus(stateEstimatesBySensor{i}, stateEstimatesBySensor{j}, t, model);
            ospaSum = ospaSum + dOspa;
            ospaCount = ospaCount + 1;
        end
    end
    if pairCount > 0
        posConsensus(t) = pairSum / pairCount;
    else
        posConsensus(t) = NaN;
    end
    if ospaCount > 0
        ospaConsensus(t) = ospaSum / ospaCount;
    else
        ospaConsensus(t) = NaN;
    end
end
end

function dist = estimateSetRmse(estA, estB, t)
muA = estA.mu{t};
muB = estB.mu{t};
if isempty(muA) && isempty(muB)
    dist = 0;
    return;
end
if isempty(muA) || isempty(muB)
    dist = NaN;
    return;
end
XA = cell2mat(cellfun(@(x) x(1:2), muA, 'UniformOutput', false));
XB = cell2mat(cellfun(@(x) x(1:2), muB, 'UniformOutput', false));
n = size(XA, 2);
m = size(XB, 2);
if n == 0 || m == 0
    dist = NaN;
    return;
end
D = zeros(n, m);
for i = 1:n
    for j = 1:m
        d = XA(:, i) - XB(:, j);
        D(i, j) = sqrt(d' * d);
    end
end
[matching, ~] = Hungarian(D);
matched = D(matching == 1);
if isempty(matched)
    dist = NaN;
    return;
end
dist = sqrt(mean(matched.^2));
end

function rmseSeries = computeSetRmseOverTime(stateEstimates, groundTruthRfs)
simLength = numel(groundTruthRfs.x);
rmseSeries = NaN(1, simLength);
for t = 1:simLength
    rmseSeries(t) = computeSetRmseAtTime(stateEstimates, groundTruthRfs, t);
end
end

function rmse = computeSetRmseAtTime(stateEstimates, groundTruthRfs, t)
truthCells = groundTruthRfs.x{t};
estCells = stateEstimates.mu{t};
if isempty(truthCells) && isempty(estCells)
    rmse = 0;
    return;
end
if isempty(truthCells) || isempty(estCells)
    rmse = NaN;
    return;
end
XT = cell2mat(cellfun(@(x) x(1:2), truthCells, 'UniformOutput', false));
XE = cell2mat(cellfun(@(x) x(1:2), estCells, 'UniformOutput', false));
n = size(XT, 2);
m = size(XE, 2);
if n == 0 || m == 0
    rmse = NaN;
    return;
end
D = zeros(n, m);
for i = 1:n
    for j = 1:m
        d = XT(:, i) - XE(:, j);
        D(i, j) = sqrt(d' * d);
    end
end
[matching, ~] = Hungarian(D);
matched = D(matching == 1);
if isempty(matched)
    rmse = NaN;
    return;
end
rmse = sqrt(mean(matched.^2));
end

function dist = estimateSetOspaConsensus(estA, estB, t, model)
muA = estA.mu{t};
SigmaA = estA.Sigma{t};
muB = estB.mu{t};
SigmaB = estB.Sigma{t};
if isempty(muA) && isempty(muB)
    dist = 0;
    return;
end
if isempty(muA) || isempty(muB)
    dist = model.ospaParameters.eC;
    return;
end
[eAB, ~] = ospa(muA, muA, SigmaA, muB, SigmaB, model.ospaParameters);
[eBA, ~] = ospa(muB, muB, SigmaB, muA, SigmaA, model.ospaParameters);
dist = 0.5 * (eAB(1) + eBA(1));
end

function offsets = localFormationOffsets(formationType, spacing, count)
if nargin < 3
    count = 3;
end
switch lower(formationType)
    case 'triangle'
        base = [0, -0.5, 0.5; 0, -0.866, -0.866];
    case 'leader3'
        base = [0, -1, -1, -2; 0, -0.7, 0.7, 0];
    otherwise
        base = [0, -1, 1; 0, -1, -1];
end
if size(base, 2) < count
    base = [base, zeros(2, count - size(base, 2))];
end
offsets = spacing * base(:, 1:count);
end

function value = getField(s, fieldName, defaultValue)
if nargin < 3
    defaultValue = [];
end
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
