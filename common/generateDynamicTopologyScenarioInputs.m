function inputs = generateDynamicTopologyScenarioInputs( ...
    presetName, seed, overrides)
% GENERATEDYNAMICTOPOLOGYSCENARIOINPUTS One-call paired experiment inputs.
%
%   inputs = generateDynamicTopologyScenarioInputs('d12-handover', 7)
%
% All experiment arms should reuse the returned inputs so trajectories,
% measurements and link uniforms remain paired.

if nargin < 1 || isempty(presetName)
    presetName = 'd12-handover';
end
if nargin < 2 || isempty(seed)
    seed = 7;
end
if nargin < 3 || isempty(overrides)
    overrides = struct();
end
rng(seed);

config = buildDynamicTopologyScenarioConfig(presetName, overrides);
[sensorTrajectories, sensorMetadata] = ...
    generateMultiFormationTrajectories(config);
[targetTrajectories, targetMetadata] = ...
    generateCorridorTargetTrajectories(config);
graphData = buildDynamicTopologyGraphs(config, sensorTrajectories);
validation = validateDynamicTopologyScenario( ...
    config, sensorTrajectories, targetTrajectories, graphData);

targetBirthStates = zeros(4, config.numberOfTargets);
for targetIdx = 1:config.numberOfTargets
    birthTime = targetMetadata.birthTimes(targetIdx);
    targetBirthStates(:, targetIdx) = ...
        targetTrajectories{targetIdx}(:, birthTime);
end
sensorMotionConfig = struct( ...
    'enabled', true, ...
    'motionType', 'CV', ...
    'processNoiseStd', 0, ...
    'initialStates', {cellfun(@(x) x(:, 1), ...
        sensorTrajectories, 'UniformOutput', false)});
targetFormationConfig = struct( ...
    'targetFormationEnabled', true, ...
    'targetFormationCount', config.numberOfTargets, ...
    'targetFormationStaggeredBirths', false, ...
    'targetFormationStartTime', 1, ...
    'targetFormationLifeSpan', config.simulationLength, ...
    'targetBirthStates', targetBirthStates);

sensorCount = config.numberOfSensors;
model = generateMultisensorModel( ...
    sensorCount, config.clutterRate * ones(1, sensorCount), ...
    config.detectionProbability * ones(1, sensorCount), ...
    config.measurementNoiseStd * ones(1, sensorCount), ...
    'GA', 'LBP', 'Formation', sensorMotionConfig, ...
    targetFormationConfig);
model.simulationLength = config.simulationLength;
model.survivalProbability = config.survivalProbability;
model.ospaParameters.eC = config.ospaPositionCutoff;
model.sensorCommRange = config.commRange;
model.sensorFovEnabled = true;
model.sensorFovHalfAngleDeg = ...
    config.fovHalfAngleDeg * ones(1, sensorCount);
model.sensorFovRange = config.fovRange * ones(1, sensorCount);
model.sensorFovHeadingRad = buildSensorFovHeadingSchedule( ...
    config, sensorTrajectories);
model.sensorQuality = config.sensorQuality;
model.observationSpaceLimits = config.regionLimits;
model.observationSpaceVolume = prod( ...
    config.regionLimits(:, 2) - config.regionLimits(:, 1));
model.clutterPerUnitVolume = model.clutterRate / ...
    model.observationSpaceVolume;
model.explicitSensorTrajectories = sensorTrajectories;
model.explicitTargetTrajectories = targetTrajectories;
model.birthTimeByLocation = targetMetadata.birthTimes;
model.rB = config.birthProbability * ...
    ones(config.numberOfTargets, 1);
model.SigmaB = repmat( ...
    {diag(config.birthCovarianceDiagonal)}, ...
    config.numberOfTargets, 1);
for targetIdx = 1:config.numberOfTargets
    model.muB{targetIdx} = targetBirthStates(:, targetIdx);
    model.birthParameters(targetIdx).r = model.rB(targetIdx);
    model.birthParameters(targetIdx).mu = model.muB(targetIdx);
    model.birthParameters(targetIdx).Sigma = model.SigmaB(targetIdx);
    model.birthParameters(targetIdx).trajectory = ...
        nan(model.xDimension, config.simulationLength);
    model.birthParameters(targetIdx).timestamps = ...
        zeros(1, config.simulationLength);
end

scenarioMetadata = struct();
scenarioMetadata.presetName = config.presetName;
scenarioMetadata.seed = seed;
scenarioMetadata.config = config;
scenarioMetadata.sensor = sensorMetadata;
scenarioMetadata.target = targetMetadata;
scenarioMetadata.targetTrajectories = targetTrajectories;
scenarioMetadata.physicalAdjacency = graphData.physicalAdjacency;
scenarioMetadata.staticAdjacency = graphData.staticAdjacency;
scenarioMetadata.candidateAdjacency = graphData.candidateAdjacency;
scenarioMetadata.candidateMetadata = graphData.candidateMetadata;
model.dynamicTopologyScenario = scenarioMetadata;

[groundTruth, measurements, groundTruthRfs, generatedSensors] = ...
    generateMultisensorGroundTruth(model);
if ~isequal(size(generatedSensors), size(sensorTrajectories))
    error('Explicit sensor trajectories were not preserved by the generator.');
end
model.sensorTrajectories = generatedSensors;

[pDropByEdge, linkMetadata] = ...
    buildDynamicTopologyLinkSchedule(config, graphData);
commConfig = struct();
commConfig.forceDelivery = config.forceDelivery;
commConfig.pDrop = 0;
commConfig.pDropBySensor = zeros(1, sensorCount);
commConfig.pDropByEdge = pDropByEdge;
commConfig.linkUniforms = rand( ...
    sensorCount, sensorCount, config.simulationLength);

inputs = struct();
inputs.seed = seed;
inputs.config = config;
inputs.model = model;
inputs.measurements = measurements;
inputs.groundTruth = groundTruth;
inputs.groundTruthRfs = groundTruthRfs;
inputs.sensorTrajectories = generatedSensors;
inputs.targetTrajectories = targetTrajectories;
inputs.neighborMap = graphData.staticNeighborMap;
inputs.commConfig = commConfig;
inputs.graphData = graphData;
inputs.linkMetadata = linkMetadata;
inputs.validation = validation;
end
