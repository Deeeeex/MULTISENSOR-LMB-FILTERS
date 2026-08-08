function result = runFormationB4V51RuntimeSmoke()
% RUNFORMATIONB4V51RUNTIMESMOKE Eight-step real-filter integration check.

presetName = 'm24-formation-fov-convoy';
seed = 41;
timeCount = 8;
inputs = generateDynamicTopologyScenarioInputs(presetName, seed);
inputs = truncateInputs(inputs, timeCount);
identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
[inputs.commConfig.linkUniforms, ~] = ...
    materializePhysicalUidDirectedDeliveryUniforms( ...
        51008, identity.sensorPhysicalUids, timeCount);
model = removeRealizedTargetTruth(inputs.model);
protocol = getFormationB4V51RuntimeProtocol();
triggerConfig = buildFormationB4V51FixedTriggerConfig( ...
    protocol.candidateArmId, inputs.config.numberOfSensors);
context = buildFormationB4V51PairedTrackingDevelopmentContext( ...
    presetName, seed, protocol.candidateArmId);

rng(51009, 'twister');
[~, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
    model, inputs.measurements, inputs.sensorTrajectories, ...
    inputs.neighborMap, inputs.commConfig, triggerConfig, context);

nodeCount = inputs.config.numberOfSensors;
messageCount = reshape(sum(sum( ...
    logical(diagnostics.attempted), 1), 2), 1, []);
pulseTimes = 1:protocol.period:timeCount;
nonpulseTimes = setdiff(1:timeCount, pulseTimes);
deferredCrossEdges = zeros(size(pulseTimes));
previousPulseKnown = false(size(pulseTimes));
temporalStrong = false(size(pulseTimes));
cycleSelected = false(size(pulseTimes));
cycleFallbackReason = repmat({''}, size(pulseTimes));
formationRetentionDebt = cell(size(pulseTimes));
requestedDeferredFormationMask = cell(size(pulseTimes));
for pulseIdx = 1:numel(pulseTimes)
    schedule = diagnostics.topologyPolicyScheduleCertificate{ ...
        pulseTimes(pulseIdx)};
    gate = schedule.retentionGateDetails;
    deferredCrossEdges(pulseIdx) = gate.deferredCrossEdgeCount;
    previousPulseKnown(pulseIdx) = gate.previousPulseKnown;
    temporalStrong(pulseIdx) = gate.temporalSensorStrong && ...
        gate.temporalFormationStrong;
    cycleSelected(pulseIdx) = schedule.cycleSelected;
    cycleFallbackReason{pulseIdx} = schedule.fallbackReason;
    formationRetentionDebt{pulseIdx} = ...
        gate.formationRetentionDebtFraction;
    requestedDeferredFormationMask{pulseIdx} = ...
        gate.requestedDeferredFormationMask;
end

passed = all(messageCount(pulseTimes) <= 2 * nodeCount) && ...
    all(messageCount(pulseTimes) >= nodeCount) && ...
    all(messageCount(nonpulseTimes) == nodeCount) && ...
    ~previousPulseKnown(1) && previousPulseKnown(2) && ...
    all(temporalStrong) && ...
    all(cellfun(@(weights) all(abs(sum(weights, 2) - 1) < 1e-12), ...
        diagnostics.topologyPolicyFusionWeightMatrix));
if ~passed
    error('FormationB4V51Smoke:RuntimeFailed', ...
        'The V51 real-filter runtime smoke failed.');
end

result = struct();
result.presetName = presetName;
result.seed = seed;
result.timeCount = timeCount;
result.messageCountByTime = messageCount;
result.deferredCrossEdgesByPulse = deferredCrossEdges;
result.previousPulseKnownByPulse = previousPulseKnown;
result.temporalStrongByPulse = temporalStrong;
result.cycleSelectedByPulse = cycleSelected;
result.cycleFallbackReasonByPulse = cycleFallbackReason;
result.formationRetentionDebtByPulse = formationRetentionDebt;
result.requestedDeferredFormationMaskByPulse = ...
    requestedDeferredFormationMask;
result.totalAttemptedMessageCount = ...
    nnz(diagnostics.attempted);
result.totalDeliveredMessageCount = ...
    nnz(diagnostics.delivered);
result.trackingOutcomeScored = false;
result.passed = passed;
fprintf(['V51 runtime smoke: messages=%s deferred=%s ', ...
    'temporal-strong=%s cycles=%s fallback=%s\n'], ...
    mat2str(messageCount), mat2str(deferredCrossEdges), ...
    mat2str(temporalStrong), mat2str(cycleSelected), ...
    strjoin(cycleFallbackReason, ','));
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
