function result = runFormationB4V53RuntimeSmoke()
% RUNFORMATIONB4V53RUNTIMESMOKE Eight-step real-filter integration check.

presetName = 'm24-formation-fov-convoy';
seed = 41;
timeCount = 8;
inputs = generateDynamicTopologyScenarioInputs(presetName, seed);
inputs = truncateInputs(inputs, timeCount);
identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
[inputs.commConfig.linkUniforms, ~] = ...
    materializePhysicalUidDirectedDeliveryUniforms( ...
        53008, identity.sensorPhysicalUids, timeCount);
model = removeRealizedTargetTruth(inputs.model);
protocol = getFormationB4V53RuntimeProtocol();
triggerConfig = buildFormationB4V53FixedTriggerConfig( ...
    protocol.candidateArmId, inputs.config.numberOfSensors);
context = buildFormationB4V53PairedTrackingDevelopmentContext( ...
    presetName, seed, protocol.candidateArmId);

rng(53009, 'twister');
[~, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
    model, inputs.measurements, inputs.sensorTrajectories, ...
    inputs.neighborMap, inputs.commConfig, triggerConfig, context);

nodeCount = inputs.config.numberOfSensors;
messageCount = reshape(sum(sum( ...
    logical(diagnostics.attempted), 1), 2), 1, []);
pulseTimes = 1:protocol.period:timeCount;
nonpulseTimes = setdiff(1:timeCount, pulseTimes);
deferredCrossEdges = zeros(size(pulseTimes));
deferredFormationCount = zeros(size(pulseTimes));
previousPulseKnown = false(size(pulseTimes));
temporalStrong = false(size(pulseTimes));
jointRetentionRisk = zeros(size(pulseTimes));
for pulseIdx = 1:numel(pulseTimes)
    schedule = diagnostics.topologyPolicyScheduleCertificate{ ...
        pulseTimes(pulseIdx)};
    gate = schedule.exactSelectiveGateDetails;
    deferredCrossEdges(pulseIdx) = gate.deferredCrossEdgeCount;
    deferredFormationCount(pulseIdx) = ...
        numel(gate.selectedDeferredFormationIds);
    previousPulseKnown(pulseIdx) = gate.previousPulseKnown;
    temporalStrong(pulseIdx) = gate.temporalSensorStrong && ...
        gate.temporalFormationStrong;
    jointRetentionRisk(pulseIdx) = gate.jointRetentionRisk;
end

passed = all(messageCount(pulseTimes) <= 2 * nodeCount) && ...
    all(messageCount(pulseTimes) >= nodeCount) && ...
    all(messageCount(nonpulseTimes) == nodeCount) && ...
    ~previousPulseKnown(1) && previousPulseKnown(2) && ...
    all(temporalStrong) && ...
    all(cellfun(@(weights) all(abs(sum(weights, 2) - 1) < 1e-12), ...
        diagnostics.topologyPolicyFusionWeightMatrix));
if ~passed
    error('FormationB4V53Smoke:RuntimeFailed', ...
        'The V53 real-filter runtime smoke failed.');
end

result = struct();
result.presetName = presetName;
result.seed = seed;
result.timeCount = timeCount;
result.messageCountByTime = messageCount;
result.deferredCrossEdgesByPulse = deferredCrossEdges;
result.deferredFormationCountByPulse = deferredFormationCount;
result.previousPulseKnownByPulse = previousPulseKnown;
result.temporalStrongByPulse = temporalStrong;
result.jointRetentionRiskByPulse = jointRetentionRisk;
result.totalAttemptedMessageCount = nnz(diagnostics.attempted);
result.totalDeliveredMessageCount = nnz(diagnostics.delivered);
result.trackingOutcomeScored = false;
result.passed = passed;
fprintf(['V53 runtime smoke: messages=%s deferred-cross=%s ', ...
    'deferred-formations=%s temporal-strong=%s risk=%s\n'], ...
    mat2str(messageCount), mat2str(deferredCrossEdges), ...
    mat2str(deferredFormationCount), mat2str(temporalStrong), ...
    mat2str(jointRetentionRisk, 4));
end

function inputs = truncateInputs(inputs, timeCount)
inputs.model.simulationLength = timeCount;
if isfield(inputs.model, 'dynamicTopologyScenario') && ...
        isfield(inputs.model.dynamicTopologyScenario, 'config')
    inputs.model.dynamicTopologyScenario.config.simulationLength = ...
        timeCount;
end
inputs.config.simulationLength = timeCount;
inputs.measurements = inputs.measurements(:, 1:timeCount);
for sensorIdx = 1:numel(inputs.sensorTrajectories)
    inputs.sensorTrajectories{sensorIdx} = ...
        inputs.sensorTrajectories{sensorIdx}(:, 1:timeCount);
end
fields = {'pDropByEdge', 'linkUniforms'};
for fieldIdx = 1:numel(fields)
    name = fields{fieldIdx};
    if isfield(inputs.commConfig, name) && ...
            ndims(inputs.commConfig.(name)) >= 3
        value = inputs.commConfig.(name);
        inputs.commConfig.(name) = value(:, :, 1:timeCount);
    end
end
if isfield(inputs.commConfig, 'nodeActiveSchedule') && ...
        size(inputs.commConfig.nodeActiveSchedule, 2) >= timeCount
    inputs.commConfig.nodeActiveSchedule = ...
        inputs.commConfig.nodeActiveSchedule(:, 1:timeCount);
end
end

function model = removeRealizedTargetTruth(model)
if isfield(model, 'explicitTargetTrajectories')
    model = rmfield(model, 'explicitTargetTrajectories');
end
fields = {'targetTrajectories', 'target'};
for fieldIdx = 1:numel(fields)
    if isfield(model.dynamicTopologyScenario, fields{fieldIdx})
        model.dynamicTopologyScenario = rmfield( ...
            model.dynamicTopologyScenario, fields{fieldIdx});
    end
end
end
