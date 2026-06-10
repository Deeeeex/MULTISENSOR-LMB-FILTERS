function [reportPath, summary] = ...
    runMultisensorFilters_formation_4plus4_DualThresholdEventTriggerCompare( ...
        numberOfTrials, baseSeed, useFixedSeed, writeReport, ...
        thresholdProfile, armSelection, experimentOverrides)
% RUNMULTISENSORFILTERS_FORMATION_4PLUS4_DUALTHRESHOLDEVENTTRIGGERCOMPARE
% Paired GA-LMB comparison for three-level posterior communication.

close all; clc;
scriptDir = fileparts(mfilename('fullpath'));
projectRoot = resolveProjectRoot(scriptDir);
addpath(projectRoot);
addpath(scriptDir);
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
if nargin < 4 || isempty(writeReport)
    writeReport = true;
end
if nargin < 5 || isempty(thresholdProfile)
    thresholdProfile = 'default';
end
if nargin < 6
    armSelection = [];
end
if nargin < 7 || isempty(experimentOverrides)
    experimentOverrides = struct();
end

scenarioConfig = buildScenarioConfig(experimentOverrides);
calibration = getField(experimentOverrides, 'calibration', []);
if isempty(calibration)
    calibrationSeed = baseSeed + 1;
    [calibrationModel, calibrationMeasurements, ~, calibrationTrajectories, ...
        calibrationComm, neighborMap] = buildTrialInputs( ...
            calibrationSeed, scenarioConfig);
    calibrationComm.forceDelivery = true;
    calibrationTrigger = buildBaseTriggerConfig();
    calibrationTrigger.eventPolicy = 'alwaysHeavy';
    calibrationTrigger.linkGateEnabled = false;
    [~, calibrationDiagnostics] = runEventTriggeredDistributedLmbFilter( ...
        calibrationModel, calibrationMeasurements, calibrationTrajectories, ...
        neighborMap, calibrationComm, calibrationTrigger);
    calibration = calibrateThresholds(calibrationDiagnostics);
end

arms = buildArms( ...
    calibration, thresholdProfile, ...
    getField(experimentOverrides, 'includeBalancedCompatibility', false), ...
    experimentOverrides);
arms = selectArms(arms, armSelection);
armNames = {arms.name};
numberOfArms = numel(arms);
numberOfSensors = scenarioConfig.numberOfSensors;

eOspa = zeros(numberOfTrials, numberOfSensors, numberOfArms);
hOspa = zeros(numberOfTrials, numberOfSensors, numberOfArms);
rmse = NaN(numberOfTrials, numberOfSensors, numberOfArms);
cardinalityError = zeros(numberOfTrials, numberOfSensors, numberOfArms);
consensusOspa = zeros(numberOfTrials, numberOfArms);
consensusPosition = NaN(numberOfTrials, numberOfArms);
consensusCardinality = zeros(numberOfTrials, numberOfArms);
runtimeSeconds = zeros(numberOfTrials, numberOfArms);
payloadBytes = zeros(numberOfTrials, numberOfArms);
payloadScalars = zeros(numberOfTrials, numberOfArms);
triggerRate = zeros(numberOfTrials, numberOfArms);
lightRate = zeros(numberOfTrials, numberOfArms);
heavyRate = zeros(numberOfTrials, numberOfArms);
deliveryRate = zeros(numberOfTrials, numberOfArms);
downgradeCount = zeros(numberOfTrials, numberOfArms);
staleFusionCount = zeros(numberOfTrials, numberOfArms);
meanStaleFusionAge = zeros(numberOfTrials, numberOfArms);
labelHeartbeatCount = zeros(numberOfTrials, numberOfArms);
algebraicConnectivity = zeros(numberOfTrials, numberOfArms);
undirectedEdgeCount = zeros(numberOfTrials, numberOfArms);
pDropBySensorTrials = zeros(numberOfTrials, numberOfSensors);
trialSeeds = NaN(1, numberOfTrials);

for trialIdx = 1:numberOfTrials
    if useFixedSeed
        trialSeed = baseSeed + trialIdx;
    else
        trialSeed = randi(2^31 - 1);
    end
    trialSeeds(trialIdx) = trialSeed;
    fprintf('Event-trigger trial %d/%d (seed=%d)\n', ...
        trialIdx, numberOfTrials, trialSeed);
    [model, measurements, groundTruthRfs, sensorTrajectories, ...
        commConfig, neighborMap] = buildTrialInputs( ...
            trialSeed, scenarioConfig);
    pDropBySensorTrials(trialIdx, :) = commConfig.pDropBySensor;

    for armIdx = 1:numberOfArms
        fprintf('  Arm %d/%d: %s\n', ...
            armIdx, numberOfArms, arms(armIdx).name);
        runtimeStart = tic();
        [stateEstimatesBySensor, communicationDiagnostics] = ...
            runEventTriggeredDistributedLmbFilter( ...
                model, measurements, sensorTrajectories, neighborMap, ...
                commConfig, arms(armIdx).triggerConfig);
        runtimeSeconds(trialIdx, armIdx) = toc(runtimeStart);

        for sensorIdx = 1:numberOfSensors
            [eSeries, hSeries, cardinalitySeries] = ...
                computeSimulationOspa( ...
                    model, groundTruthRfs, ...
                    stateEstimatesBySensor{sensorIdx});
            eOspa(trialIdx, sensorIdx, armIdx) = mean(eSeries);
            hOspa(trialIdx, sensorIdx, armIdx) = mean(hSeries);
            rmse(trialIdx, sensorIdx, armIdx) = mean( ...
                computeSetRmseOverTime( ...
                    stateEstimatesBySensor{sensorIdx}, groundTruthRfs), ...
                'omitnan');
            cardinalityError(trialIdx, sensorIdx, armIdx) = mean( ...
                abs(cardinalitySeries - groundTruthRfs.cardinality));
        end

        [positionSeries, cardinalitySeries, ospaSeries] = ...
            computeDistributedConsensusMetrics( ...
                stateEstimatesBySensor, model);
        consensusOspa(trialIdx, armIdx) = mean(ospaSeries);
        consensusPosition(trialIdx, armIdx) = ...
            mean(positionSeries, 'omitnan');
        consensusCardinality(trialIdx, armIdx) = ...
            mean(cardinalitySeries);

        communicationSummary = communicationDiagnostics.summary;
        payloadBytes(trialIdx, armIdx) = communicationSummary.payloadBytes;
        payloadScalars(trialIdx, armIdx) = ...
            communicationSummary.payloadScalars;
        triggerRate(trialIdx, armIdx) = communicationSummary.triggerRate;
        lightRate(trialIdx, armIdx) = communicationSummary.lightRate;
        heavyRate(trialIdx, armIdx) = communicationSummary.heavyRate;
        deliveryRate(trialIdx, armIdx) = communicationSummary.deliveryRate;
        downgradeCount(trialIdx, armIdx) = ...
            communicationSummary.downgradeCount;
        staleFusionCount(trialIdx, armIdx) = getField( ...
            communicationSummary, 'staleFusionCount', 0);
        meanStaleFusionAge(trialIdx, armIdx) = getField( ...
            communicationSummary, 'meanStaleFusionAge', 0);
        labelHeartbeatCount(trialIdx, armIdx) = getField( ...
            communicationSummary, 'labelHeartbeatCount', 0);
        algebraicConnectivity(trialIdx, armIdx) = getField( ...
            communicationSummary, 'meanAlgebraicConnectivity', 0);
        undirectedEdgeCount(trialIdx, armIdx) = getField( ...
            communicationSummary, 'meanUndirectedEdgeCount', 0);
        fprintf('    bytes=%.0f trigger=%.3f E-OSPA=%.3f runtime=%.2fs\n', ...
            payloadBytes(trialIdx, armIdx), ...
            triggerRate(trialIdx, armIdx), ...
            mean(eOspa(trialIdx, :, armIdx)), ...
            runtimeSeconds(trialIdx, armIdx));
    end
end

summary = struct();
summary.armNames = armNames;
summary.arms = arms;
summary.numberOfTrials = numberOfTrials;
summary.trialSeeds = trialSeeds;
summary.scenarioConfig = scenarioConfig;
summary.calibration = calibration;
summary.meanPDropBySensor = mean(pDropBySensorTrials, 1);
summary.local.eOspa = squeeze(mean(eOspa, 1));
summary.local.hOspa = squeeze(mean(hOspa, 1));
summary.local.rmse = squeeze(mean(rmse, 1, 'omitnan'));
summary.local.cardinalityError = squeeze(mean(cardinalityError, 1));
summary.local.meanAcrossSensors.eOspa = ...
    computeArmMeansAcrossTrialsAndSensors(eOspa, false);
summary.local.meanAcrossSensors.hOspa = ...
    computeArmMeansAcrossTrialsAndSensors(hOspa, false);
summary.local.meanAcrossSensors.rmse = ...
    computeArmMeansAcrossTrialsAndSensors(rmse, true);
summary.local.meanAcrossSensors.cardinalityError = ...
    computeArmMeansAcrossTrialsAndSensors(cardinalityError, false);
summary.consensus.ospa = mean(consensusOspa, 1);
summary.consensus.position = mean(consensusPosition, 1, 'omitnan');
summary.consensus.cardinality = mean(consensusCardinality, 1);
summary.communication.payloadBytes = mean(payloadBytes, 1);
summary.communication.payloadScalars = mean(payloadScalars, 1);
summary.communication.triggerRate = mean(triggerRate, 1);
summary.communication.lightRate = mean(lightRate, 1);
summary.communication.heavyRate = mean(heavyRate, 1);
summary.communication.deliveryRate = mean(deliveryRate, 1);
summary.communication.downgradeCount = mean(downgradeCount, 1);
summary.communication.staleFusionCount = mean(staleFusionCount, 1);
summary.communication.meanStaleFusionAge = mean(meanStaleFusionAge, 1);
summary.communication.labelHeartbeatCount = mean(labelHeartbeatCount, 1);
summary.topology.algebraicConnectivity = mean(algebraicConnectivity, 1);
summary.topology.undirectedEdgeCount = mean(undirectedEdgeCount, 1);
summary.runtime.meanSeconds = mean(runtimeSeconds, 1);
summary.trials.localEOspa = eOspa;
summary.trials.consensusOspa = consensusOspa;
summary.trials.consensusPosition = consensusPosition;
summary.trials.consensusCardinality = consensusCardinality;
summary.trials.payloadBytes = payloadBytes;
summary.trials.triggerRate = triggerRate;
summary.trials.staleFusionCount = staleFusionCount;
summary.trials.labelHeartbeatCount = labelHeartbeatCount;
summary.trials.algebraicConnectivity = algebraicConnectivity;
summary.trials.undirectedEdgeCount = undirectedEdgeCount;
summary.acceptance = evaluateAcceptance(summary);
summary.pareto = computePareto(summary);

printSummary(summary);
reportPath = '';
if writeReport
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    reportName = sprintf( ...
        'GA_DUAL_THRESHOLD_EVENT_TRIGGER_N%d_SEED%d_%s.md', ...
        numberOfTrials, baseSeed, timestamp);
    reportPath = fullfile(scriptDir, reportName);
    writeResearchReport(reportPath, summary);
    fprintf('Report written: %s\n', reportPath);
end
end

function config = buildScenarioConfig(overrides)
config = struct();
config.numberOfSensors = 8;
config.simulationLength = getField(overrides, 'simulationLength', 100);
config.clutterRates = 3 * ones(1, config.numberOfSensors);
config.detectionProbabilities = 0.9 * ones(1, config.numberOfSensors);
config.measurementNoiseStd = 3 * ones(1, config.numberOfSensors);
config.sensorCommRange = 150;
config.pDrop = 0.2;
config.pDropLevels = [0, 0.1, 0.2, 0.5];
config.pDropLevelCounts = [1, 4, 1, 2];
config.pDropByEdge = getField(overrides, 'pDropByEdge', []);
config.outageSchedule = getField(overrides, 'outageSchedule', []);
config.sensorMotionConfig = struct( ...
    'enabled', true, ...
    'motionType', 'CV', ...
    'processNoiseStd', 0.0, ...
    'initialStates', {buildSensorInitialStates()});
targetBirthStates = buildTargetBirthStates();
config.targetFormationConfig = struct( ...
    'targetFormationEnabled', true, ...
    'targetFormationStaggeredBirths', true, ...
    'targetFormationBirthInterval', 8, ...
    'targetFormationStartTime', 1, ...
    'targetFormationLifeSpan', config.simulationLength, ...
    'targetBirthStates', targetBirthStates, ...
    'targetFormationCount', size(targetBirthStates, 2));
end

function [model, measurements, groundTruthRfs, sensorTrajectories, ...
    commConfig, neighborMap] = buildTrialInputs(seed, scenarioConfig)
rng(seed);
model = generateMultisensorModel( ...
    scenarioConfig.numberOfSensors, scenarioConfig.clutterRates, ...
    scenarioConfig.detectionProbabilities, ...
    scenarioConfig.measurementNoiseStd, 'GA', 'LBP', 'Formation', ...
    scenarioConfig.sensorMotionConfig, ...
    scenarioConfig.targetFormationConfig);
model.simulationLength = scenarioConfig.simulationLength;
model.sensorCommRange = scenarioConfig.sensorCommRange;
model.fusionWeighting = 'Metropolis';
model.sensorFovEnabled = true;
model.sensorFovHalfAngleDeg = 60;
model.sensorFovRange = 60000;
[~, measurements, groundTruthRfs, sensorTrajectories] = ...
    generateMultisensorGroundTruth(model);
model.sensorTrajectories = sensorTrajectories;

assignedDropRates = expandTieredRates( ...
    scenarioConfig.pDropLevels, ...
    scenarioConfig.pDropLevelCounts);
assignedDropRates = assignedDropRates( ...
    randperm(scenarioConfig.numberOfSensors));
commConfig = struct();
commConfig.pDrop = scenarioConfig.pDrop;
commConfig.pDropBySensor = assignedDropRates;
commConfig.pDropByEdge = scenarioConfig.pDropByEdge;
commConfig.outageSchedule = scenarioConfig.outageSchedule;
commConfig.linkUniforms = rand( ...
    scenarioConfig.numberOfSensors, ...
    scenarioConfig.numberOfSensors, ...
    scenarioConfig.simulationLength);
neighborMap = buildNeighborMap4Plus4(scenarioConfig.numberOfSensors);
end

function triggerConfig = buildBaseTriggerConfig()
triggerConfig = struct( ...
    'eventPolicy', 'dual', ...
    'criterionMode', 'multi', ...
    'criterionWeights', [0.20, 0.20, 0.35, 0.25], ...
    'thresholdLow', 0.25, ...
    'thresholdHigh', 0.60, ...
    'linkGateEnabled', true, ...
    'poorLinkThreshold', 0.35, ...
    'moderateLinkThreshold', 0.70, ...
    'recentWindow', 5, ...
    'maxReferenceAge', 10, ...
    'forceInitialHeavy', true, ...
    'forceLabelChangeHeavy', true, ...
    'forceLabelExistenceThreshold', 0.5, ...
    'activeExistenceThreshold', 1e-2, ...
    'payloadExistenceThreshold', 1e-2, ...
    'fusionWeightMode', 'metropolis');
end

function calibration = calibrateThresholds(diagnostics)
utilitySamples = positiveFinite( ...
    diagnostics.summary.utilitySamples);
informationSamples = positiveFinite( ...
    diagnostics.summary.informationGainSamples);
calibration.sampleCount.utility = numel(utilitySamples);
calibration.sampleCount.informationGain = numel(informationSamples);
calibration.multi.loose = quantilePair(utilitySamples, [0.40, 0.80]);
calibration.multi.default = quantilePair(utilitySamples, [0.60, 0.90]);
calibration.multi.strict = quantilePair(utilitySamples, [0.75, 0.95]);
calibration.information.loose = ...
    quantilePair(informationSamples, [0.40, 0.80]);
calibration.information.default = ...
    quantilePair(informationSamples, [0.60, 0.90]);
calibration.information.strict = ...
    quantilePair(informationSamples, [0.75, 0.95]);
end

function arms = buildArms( ...
    calibration, thresholdProfile, includeBalanced, experimentOverrides)
base = buildBaseTriggerConfig();
profile = lower(char(thresholdProfile));
if strcmp(profile, 'all')
    fullProfiles = {'loose', 'default', 'strict'};
    controlProfile = 'default';
else
    fullProfiles = {profile};
    controlProfile = profile;
end
if ~isfield(calibration.multi, controlProfile)
    error('Unknown threshold profile: %s', thresholdProfile);
end

arms = repmat(struct('name', '', 'purpose', '', ...
    'triggerConfig', struct()), 1, 5 + numel(fullProfiles));
cfg = base;
cfg.eventPolicy = 'none';
cfg.linkGateEnabled = false;
arms(1) = makeArm('Local only', ...
    'No posterior communication.', cfg);

cfg = base;
cfg.eventPolicy = 'alwaysHeavy';
cfg.linkGateEnabled = false;
arms(2) = makeArm('Periodic full posterior', ...
    'Full GM-LMB on every directed edge and step.', cfg);

cfg = applyThresholdPair(base, calibration.information.(controlProfile));
cfg.eventPolicy = 'singleHeavy';
cfg.criterionMode = 'informationOnly';
cfg.linkGateEnabled = false;
arms(3) = makeArm('Single-threshold information, full payload', ...
    'Information-gain-only trigger with full messages.', cfg);

cfg = applyThresholdPair(base, calibration.information.(controlProfile));
cfg.eventPolicy = 'dual';
cfg.criterionMode = 'informationOnly';
cfg.linkGateEnabled = false;
arms(4) = makeArm('Dual-threshold information', ...
    'Information-gain-only light/heavy trigger.', cfg);

cfg = applyThresholdPair(base, calibration.multi.(controlProfile));
cfg.eventPolicy = 'dual';
cfg.criterionMode = 'multi';
cfg.linkGateEnabled = false;
arms(5) = makeArm('Multi-indicator without link gate', ...
    'Multi-indicator light/heavy trigger without link downgrading.', cfg);

for profileIdx = 1:numel(fullProfiles)
    currentProfile = fullProfiles{profileIdx};
    cfg = applyThresholdPair(base, calibration.multi.(currentProfile));
    cfg.eventPolicy = 'dual';
    cfg.criterionMode = 'multi';
    cfg.linkGateEnabled = true;
    arms(5 + profileIdx) = makeArm( ...
        sprintf('Full method (%s)', currentProfile), ...
        'Multi-indicator dual-threshold trigger with link gating.', cfg);
end

if includeBalanced
    cfg = arms(end).triggerConfig;
    cfg.fusionWeightMode = 'balanced';
    cfg.adaptiveFusionConfig = buildBalancedCompatibilityConfig();
    arms(end+1) = makeArm('Full method + Balanced compatibility', ...
        'Optional no-EMA/no-floor adaptive fusion compatibility arm.', cfg);
end

if getField(experimentOverrides, 'includeStaleCacheVariants', false)
    staleAges = getField(experimentOverrides, 'staleCacheAges', [1, 3, 5]);
    staleAges = unique(round(reshape(staleAges, 1, [])));
    staleAges = staleAges(staleAges > 0);
    for ageIdx = 1:numel(staleAges)
        cfg = buildStaleCacheVariantConfig( ...
            base, calibration.multi.(controlProfile), ...
            staleAges(ageIdx), 0);
        arms(end+1) = makeArm( ...
            sprintf('Full method + stale cache K=%d', staleAges(ageIdx)), ...
            'Full method plus bounded predicted stale-neighbor fusion.', ...
            cfg); %#ok<AGROW>
    end

    heartbeatAges = getField( ...
        experimentOverrides, 'labelHeartbeatAges', [5, 10]);
    heartbeatAges = unique(round(reshape(heartbeatAges, 1, [])));
    heartbeatAges = heartbeatAges(heartbeatAges > 0);
    heartbeatBaseAge = getField( ...
        experimentOverrides, 'staleHeartbeatBaseAge', 3);
    for heartbeatIdx = 1:numel(heartbeatAges)
        cfg = buildStaleCacheVariantConfig( ...
            base, calibration.multi.(controlProfile), ...
            heartbeatBaseAge, heartbeatAges(heartbeatIdx));
        arms(end+1) = makeArm( ...
            sprintf('Full method + stale K=%d + heartbeat M=%d', ...
                heartbeatBaseAge, heartbeatAges(heartbeatIdx)), ...
            'Bounded stale-neighbor fusion plus label-level light heartbeat.', ...
            cfg); %#ok<AGROW>
    end
end

if getField(experimentOverrides, 'includeMissingWeightVariants', false)
    cfg = applyThresholdPair(base, calibration.multi.(controlProfile));
    cfg.eventPolicy = 'dual';
    cfg.criterionMode = 'multi';
    cfg.linkGateEnabled = true;
    cfg.missingNeighborWeightMode = 'self';
    arms(end+1) = makeArm('Full method + self-mass missing edges', ...
        'Missing directed-edge Metropolis mass is assigned to local posterior.', ...
        cfg);
end

if getField(experimentOverrides, 'includeDynamicTopologyVariants', false)
    staticEdgeBudget = countUndirectedEdgesFromNeighborMap( ...
        buildNeighborMap4Plus4(8));

    cfg = base;
    cfg.eventPolicy = 'alwaysHeavy';
    cfg.linkGateEnabled = false;
    cfg.dynamicTopologyEnabled = true;
    cfg.dynamicTopologyEdgeBudget = staticEdgeBudget;
    cfg.topologyMinAlgebraicConnectivity = -1;
    arms(end+1) = makeArm('Periodic full posterior + dynamic topology', ...
        'Full GM-LMB on a budget-preserving rewired topology.', ...
        cfg);

    cfg = applyThresholdPair(base, calibration.multi.(controlProfile));
    cfg.eventPolicy = 'dual';
    cfg.criterionMode = 'multi';
    cfg.linkGateEnabled = true;
    cfg.dynamicTopologyEnabled = true;
    cfg.dynamicTopologyEdgeBudget = staticEdgeBudget;
    cfg.topologyMinAlgebraicConnectivity = -1;
    arms(end+1) = makeArm('Full method + dynamic topology', ...
        'Budget-preserving topology rewiring by link reliability and posterior label overlap.', ...
        cfg);
end

if getField(experimentOverrides, 'includeLightFloorVariants', false)
    staticEdgeBudget = countUndirectedEdgesFromNeighborMap( ...
        buildNeighborMap4Plus4(8));

    cfg = buildLightFloorVariantConfig( ...
        base, calibration.multi.(controlProfile), false, ...
        staticEdgeBudget, experimentOverrides);
    arms(end+1) = makeArm('Light-floor dual threshold', ...
        'Low-threshold light synchronization with sparse heavy refreshes.', ...
        cfg);

    cfg = buildLightFloorVariantConfig( ...
        base, calibration.multi.(controlProfile), true, ...
        staticEdgeBudget, experimentOverrides);
    arms(end+1) = makeArm('Light-floor dual threshold + dynamic topology', ...
        'Budget-preserving dynamic topology plus per-edge light synchronization floor.', ...
        cfg);
end
end

function cfg = buildLightFloorVariantConfig( ...
    base, thresholdPair, useDynamicTopology, staticEdgeBudget, ...
    experimentOverrides)
cfg = applyThresholdPair(base, thresholdPair);
cfg.eventPolicy = 'dual';
cfg.criterionMode = 'multi';
cfg.linkGateEnabled = true;
cfg.thresholdLow = getField(experimentOverrides, ...
    'lightFloorThresholdLow', 0.20);
cfg.thresholdHigh = getField(experimentOverrides, ...
    'lightFloorThresholdHigh', thresholdPair(2));
cfg.poorLinkThreshold = 0.0;
cfg.moderateLinkThreshold = 0.70;
cfg.forceInitialHeavy = false;
cfg.forceLabelChangeHeavy = false;
cfg.forceStaleHeavy = false;
cfg.dynamicTopologyEnabled = useDynamicTopology;
cfg.dynamicTopologyEdgeBudget = staticEdgeBudget;
cfg.topologyMinAlgebraicConnectivity = -1;
cfg.topologyReliabilityWeight = 0.55;
cfg.topologyOverlapWeight = 0.35;
cfg.topologyComplementarityWeight = 0.10;
cfg.topologyStaticEdgeBonus = getField(experimentOverrides, ...
    'lightFloorStaticEdgeBonus', 0.35);
cfg.topologyFallbackToBaseOnConnectivityFailure = true;
end

function cfg = buildStaleCacheVariantConfig(base, thresholdPair, ...
    staleAge, heartbeatAge)
cfg = applyThresholdPair(base, thresholdPair);
cfg.eventPolicy = 'dual';
cfg.criterionMode = 'multi';
cfg.linkGateEnabled = true;
cfg.useStaleNeighborCache = true;
cfg.maxStaleFusionAge = staleAge;
cfg.staleFusionWeightDecay = 0.70;
if heartbeatAge > 0
    cfg.labelHeartbeatEnabled = true;
    cfg.labelHeartbeatMaxAge = heartbeatAge;
    cfg.labelHeartbeatEventType = 1;
end
end

function cfg = buildBalancedCompatibilityConfig()
cfg = struct( ...
    'enabled', true, ...
    'method', 'factorized', ...
    'useCovariance', true, ...
    'useLinkQuality', true, ...
    'useExistenceConfidence', true, ...
    'existenceConfidenceMinScore', 0.85, ...
    'existenceConfidencePower', 2.0, ...
    'useDecoupledKla', true, ...
    'spatialDecouplingStrength', 0.5, ...
    'existenceDecouplingStrength', 0.15, ...
    'useStructureAwareKla', true, ...
    'usePosteriorStructureConsistency', false, ...
    'spatialStructureStrength', 0.45, ...
    'existenceStructureStrength', 0.08, ...
    'structureReliabilityPower', 0.30, ...
    'structureReliabilityMinScore', 0.25, ...
    'useFidFiaExistence', false, ...
    'emaAlpha', 0.0, ...
    'minWeight', 0.0, ...
    'spatialEmaAlpha', 0.0, ...
    'existenceEmaAlpha', 0.0, ...
    'spatialMinWeight', 0.0, ...
    'existenceMinWeight', 0.0);
end

function arms = selectArms(arms, armSelection)
if isempty(armSelection)
    return;
end
if isnumeric(armSelection)
    indices = unique(round(reshape(armSelection, 1, [])));
    indices = indices(indices >= 1 & indices <= numel(arms));
    arms = arms(indices);
    return;
end
queries = cellstr(armSelection);
selected = false(1, numel(arms));
for queryIdx = 1:numel(queries)
    query = lower(strtrim(queries{queryIdx}));
    for armIdx = 1:numel(arms)
        if ~isempty(strfind(lower(arms(armIdx).name), query)) %#ok<STREMP>
            selected(armIdx) = true;
        end
    end
end
if any(selected)
    arms = arms(selected);
end
end

function cfg = applyThresholdPair(cfg, pair)
cfg.thresholdLow = pair(1);
cfg.thresholdHigh = pair(2);
end

function arm = makeArm(name, purpose, triggerConfig)
arm = struct('name', name, 'purpose', purpose, ...
    'triggerConfig', triggerConfig);
end

function count = countUndirectedEdgesFromNeighborMap(neighborMap)
numberOfSensors = numel(neighborMap);
adjacency = false(numberOfSensors);
for sensorIdx = 1:numberOfSensors
    neighbors = neighborMap{sensorIdx};
    neighbors = neighbors(neighbors ~= sensorIdx);
    adjacency(sensorIdx, neighbors) = true;
end
adjacency = adjacency | adjacency';
count = nnz(triu(adjacency, 1));
end

function acceptance = evaluateAcceptance(summary)
baselineIdx = find(strcmp(summary.armNames, ...
    'Periodic full posterior'), 1);
numberOfArms = numel(summary.armNames);
acceptance = repmat(struct( ...
    'qualifiesForScaleUp', false, ...
    'byteReductionPercent', NaN, ...
    'localEOspaChangePercent', NaN, ...
    'consensusOspaChangePercent', NaN, ...
    'consensusPositionChangePercent', NaN, ...
    'consensusCardinalityChangePercent', NaN, ...
    'failureReasons', {{}}), 1, numberOfArms);
if isempty(baselineIdx)
    return;
end

for armIdx = 1:numberOfArms
    acceptance(armIdx).byteReductionPercent = percentReduction( ...
        summary.communication.payloadBytes(baselineIdx), ...
        summary.communication.payloadBytes(armIdx));
    acceptance(armIdx).localEOspaChangePercent = percentChange( ...
        summary.local.meanAcrossSensors.eOspa(baselineIdx), ...
        summary.local.meanAcrossSensors.eOspa(armIdx));
    acceptance(armIdx).consensusOspaChangePercent = percentChange( ...
        summary.consensus.ospa(baselineIdx), summary.consensus.ospa(armIdx));
    acceptance(armIdx).consensusPositionChangePercent = percentChange( ...
        summary.consensus.position(baselineIdx), ...
        summary.consensus.position(armIdx));
    acceptance(armIdx).consensusCardinalityChangePercent = percentChange( ...
        summary.consensus.cardinality(baselineIdx), ...
        summary.consensus.cardinality(armIdx));
    failures = {};
    if acceptance(armIdx).byteReductionPercent < 30
        failures{end+1} = 'payload reduction < 30%'; %#ok<AGROW>
    end
    if acceptance(armIdx).localEOspaChangePercent > 5
        failures{end+1} = 'local E-OSPA degradation > 5%'; %#ok<AGROW>
    end
    if acceptance(armIdx).consensusOspaChangePercent > 10
        failures{end+1} = 'OSPA disagreement degradation > 10%'; %#ok<AGROW>
    end
    if acceptance(armIdx).consensusPositionChangePercent > 10
        failures{end+1} = 'position disagreement degradation > 10%'; %#ok<AGROW>
    end
    if acceptance(armIdx).consensusCardinalityChangePercent > 10
        failures{end+1} = 'cardinality dispersion degradation > 10%'; %#ok<AGROW>
    end
    acceptance(armIdx).failureReasons = failures;
    acceptance(armIdx).qualifiesForScaleUp = isempty(failures);
end
end

function pareto = computePareto(summary)
bytes = summary.communication.payloadBytes;
eOspa = summary.local.meanAcrossSensors.eOspa;
numberOfArms = numel(bytes);
nondominated = true(1, numberOfArms);
for armIdx = 1:numberOfArms
    for candidateIdx = 1:numberOfArms
        if candidateIdx == armIdx
            continue;
        end
        noWorse = bytes(candidateIdx) <= bytes(armIdx) && ...
            eOspa(candidateIdx) <= eOspa(armIdx);
        strictlyBetter = bytes(candidateIdx) < bytes(armIdx) || ...
            eOspa(candidateIdx) < eOspa(armIdx);
        if noWorse && strictlyBetter
            nondominated(armIdx) = false;
            break;
        end
    end
end
pareto.nondominated = nondominated;
pareto.armNames = summary.armNames(nondominated);
end

function printSummary(summary)
fprintf('=====================================\n');
fprintf('Dual-threshold event-trigger GA-LMB (N=%d)\n', ...
    summary.numberOfTrials);
fprintf('Calibration default multi thresholds: %s\n', ...
    mat2str(summary.calibration.multi.default, 4));
fprintf('=====================================\n');
for armIdx = 1:numel(summary.armNames)
    fprintf('%s: bytes %.0f, trigger %.3f, local E-OSPA %.3f, consensus OSPA %.3f\n', ...
        summary.armNames{armIdx}, ...
        summary.communication.payloadBytes(armIdx), ...
        summary.communication.triggerRate(armIdx), ...
        summary.local.meanAcrossSensors.eOspa(armIdx), ...
        summary.consensus.ospa(armIdx));
end
end

function writeResearchReport(reportPath, summary)
fid = fopen(reportPath, 'w');
if fid < 0
    error('Could not open report path: %s', reportPath);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '# 双阈值多指标事件触发 GA-LMB 研究报告\n\n');
fprintf(fid, '- Trials: %d\n', summary.numberOfTrials);
fprintf(fid, '- Seeds: `%s`\n', mat2str(summary.trialSeeds));
fprintf(fid, '- 仿真长度: %d\n', summary.scenarioConfig.simulationLength);
fprintf(fid, '- 平均逐节点丢包率: `%s`\n\n', ...
    mat2str(summary.meanPDropBySensor, 4));
fprintf(fid, '本实验验证的组合是多指标耦合、链路门控降级和轻/重两级 LMB 后验负载；事件触发 LMB 与目标级信息增益触发已有文献先例，不作为单独新颖性主张。\n\n');
fprintf(fid, '- TSP 2022: https://doi.org/10.1109/TSP.2022.3154227\n');
fprintf(fid, '- TAES 2023: https://doi.org/10.1109/TAES.2022.3187390\n');
fprintf(fid, '- TCSII 2023: https://doi.org/10.1109/TCSII.2023.3238346\n\n');

fprintf(fid, '## 单试验全通信阈值标定\n\n');
fprintf(fid, '| Criterion | Loose | Default | Strict | Samples |\n');
fprintf(fid, '|:--|:--|:--|:--|--:|\n');
fprintf(fid, '| Multi-indicator | `%s` | `%s` | `%s` | %s |\n', ...
    formatCalibrationPair(summary.calibration, 'multi', 'loose'), ...
    formatCalibrationPair(summary.calibration, 'multi', 'default'), ...
    formatCalibrationPair(summary.calibration, 'multi', 'strict'), ...
    formatCalibrationSampleCount(summary.calibration, 'utility'));
fprintf(fid, '| Information gain | `%s` | `%s` | `%s` | %s |\n\n', ...
    formatCalibrationPair(summary.calibration, 'information', 'loose'), ...
    formatCalibrationPair(summary.calibration, 'information', 'default'), ...
    formatCalibrationPair(summary.calibration, 'information', 'strict'), ...
    formatCalibrationSampleCount(summary.calibration, 'informationGain'));

fprintf(fid, '## 通信与性能结果\n\n');
fprintf(fid, '| Arm | Scalars | Bytes | Trigger | Light | Heavy | Delivery | Downgrades | Edges | Alg. conn. | Runtime (s) | Local E-OSPA | Local RMSE | Consensus OSPA | Position disagreement | Card. dispersion |\n');
fprintf(fid, '|:--|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|--:|\n');
for armIdx = 1:numel(summary.armNames)
    fprintf(fid, '| %s | %.0f | %.0f | %.3f | %.3f | %.3f | %.3f | %.1f | %.1f | %.3f | %.3f | %.4f | %.4f | %.4f | %.4f | %.4f |\n', ...
        summary.armNames{armIdx}, ...
        summary.communication.payloadScalars(armIdx), ...
        summary.communication.payloadBytes(armIdx), ...
        summary.communication.triggerRate(armIdx), ...
        summary.communication.lightRate(armIdx), ...
        summary.communication.heavyRate(armIdx), ...
        summary.communication.deliveryRate(armIdx), ...
        summary.communication.downgradeCount(armIdx), ...
        summary.topology.undirectedEdgeCount(armIdx), ...
        summary.topology.algebraicConnectivity(armIdx), ...
        summary.runtime.meanSeconds(armIdx), ...
        summary.local.meanAcrossSensors.eOspa(armIdx), ...
        summary.local.meanAcrossSensors.rmse(armIdx), ...
        summary.consensus.ospa(armIdx), ...
        summary.consensus.position(armIdx), ...
        summary.consensus.cardinality(armIdx));
end

fprintf(fid, '\n## 30%% 通信节省升级门槛\n\n');
fprintf(fid, '| Arm | Bytes reduction | Local E-OSPA change | Consensus OSPA change | Position change | Card. change | Scale up | Failure reasons |\n');
fprintf(fid, '|:--|--:|--:|--:|--:|--:|:--:|:--|\n');
for armIdx = 1:numel(summary.armNames)
    result = summary.acceptance(armIdx);
    reasons = strjoin(result.failureReasons, '; ');
    if isempty(reasons)
        reasons = '-';
    end
    fprintf(fid, '| %s | %.1f%% | %.1f%% | %.1f%% | %.1f%% | %.1f%% | %d | %s |\n', ...
        summary.armNames{armIdx}, result.byteReductionPercent, ...
        result.localEOspaChangePercent, ...
        result.consensusOspaChangePercent, ...
        result.consensusPositionChangePercent, ...
        result.consensusCardinalityChangePercent, ...
        result.qualifiesForScaleUp, reasons);
end

fprintf(fid, '\n## Pareto 筛选\n\n');
fprintf(fid, '按估算字节数与 local E-OSPA 的二维非支配关系，当前 Pareto arms 为：`%s`。\n', ...
    strjoin(summary.pareto.armNames, '`, `'));
clear cleanup;
end

function text = formatCalibrationPair(calibration, criterionName, profileName)
text = 'n/a';
if ~isfield(calibration, criterionName)
    return;
end
criterion = calibration.(criterionName);
if ~isfield(criterion, profileName)
    return;
end
text = mat2str(criterion.(profileName), 4);
end

function text = formatCalibrationSampleCount(calibration, fieldName)
text = 'n/a';
if ~isfield(calibration, 'sampleCount') || ...
        ~isfield(calibration.sampleCount, fieldName)
    return;
end
text = sprintf('%d', calibration.sampleCount.(fieldName));
end

function values = computeArmMeansAcrossTrialsAndSensors(arrayLike, omitNaN)
numberOfArms = size(arrayLike, 3);
values = zeros(1, numberOfArms);
for armIdx = 1:numberOfArms
    armValues = reshape(arrayLike(:, :, armIdx), [], 1);
    if omitNaN
        values(armIdx) = mean(armValues, 'omitnan');
    else
        values(armIdx) = mean(armValues);
    end
end
end

function value = percentReduction(baseline, candidate)
if ~isfinite(baseline) || baseline <= eps
    value = NaN;
else
    value = 100 * (baseline - candidate) / baseline;
end
end

function value = percentChange(baseline, candidate)
if ~isfinite(baseline) || abs(baseline) <= eps
    if abs(candidate) <= eps
        value = 0;
    else
        value = inf;
    end
else
    value = 100 * (candidate - baseline) / abs(baseline);
end
end

function values = positiveFinite(values)
values = values(isfinite(values) & values > 0);
if isempty(values)
    values = [0.10, 0.40];
end
values = sort(reshape(values, 1, []));
end

function pair = quantilePair(values, probabilities)
pair = zeros(1, 2);
for idx = 1:2
    probability = min(max(probabilities(idx), 0), 1);
    position = 1 + probability * (numel(values) - 1);
    lowerIdx = floor(position);
    upperIdx = ceil(position);
    if lowerIdx == upperIdx
        pair(idx) = values(lowerIdx);
    else
        fraction = position - lowerIdx;
        pair(idx) = values(lowerIdx) * (1 - fraction) + ...
            values(upperIdx) * fraction;
    end
end
if pair(2) < pair(1)
    pair = fliplr(pair);
end
end

function rates = expandTieredRates(levels, counts)
rates = [];
for idx = 1:numel(levels)
    rates = [rates, repmat(levels(idx), 1, counts(idx))]; %#ok<AGROW>
end
end

function neighborMap = buildNeighborMap4Plus4(numberOfSensors)
if numberOfSensors ~= 8
    error('The 4+4 experiment requires eight sensors.');
end
groupA = 1:4;
groupB = 5:8;
pairings = [1 5; 2 6; 3 7; 4 8];
neighborMap = cell(1, numberOfSensors);
for idx = 1:4
    neighborMap{groupA(idx)} = unique([groupA, pairings(idx, 2)]);
    neighborMap{groupB(idx)} = unique([groupB, pairings(idx, 1)]);
end
end

function sensorInitialStates = buildSensorInitialStates()
groupCenters = [-80, -80; 35, -35];
spacing = 20;
velocity = [0.8; 0];
sensorInitialStates = cell(1, 8);
cursor = 1;
for groupIdx = 1:2
    offsets = localFormationOffsets('Leader3', spacing, 4);
    for sensorIdx = 1:4
        position = groupCenters(:, groupIdx) + offsets(:, sensorIdx);
        sensorInitialStates{cursor} = [position; velocity];
        cursor = cursor + 1;
    end
end
end

function targetBirthStates = buildTargetBirthStates()
targetCenter = [0; 0];
groupCenters = [70, 80, 70; 80, 0, -80];
groupTypes = {'Triangle', 'Triangle', 'Leader3'};
groupCounts = [3, 3, 4];
groupSpacing = [30, 25, 20];
groupSpeed = [0.45, 0.45, 0.45];
targetBirthStates = zeros(4, sum(groupCounts));
cursor = 1;
for groupIdx = 1:numel(groupCounts)
    offsets = localFormationOffsets( ...
        groupTypes{groupIdx}, groupSpacing(groupIdx), ...
        groupCounts(groupIdx));
    center = groupCenters(:, groupIdx);
    direction = targetCenter - center;
    velocity = groupSpeed(groupIdx) * direction / max(norm(direction), eps);
    for targetIdx = 1:groupCounts(groupIdx)
        targetBirthStates(:, cursor) = [ ...
            center + offsets(:, targetIdx); velocity];
        cursor = cursor + 1;
    end
end
end

function offsets = localFormationOffsets(formationType, spacing, count)
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

function projectRoot = resolveProjectRoot(scriptDir)
projectRoot = scriptDir;
for idx = 1:6
    if exist(fullfile(projectRoot, 'setPath.m'), 'file')
        return;
    end
    projectRoot = fileparts(projectRoot);
end
end

function value = getField(s, fieldName, defaultValue)
if isstruct(s) && isfield(s, fieldName)
    value = s.(fieldName);
else
    value = defaultValue;
end
end
