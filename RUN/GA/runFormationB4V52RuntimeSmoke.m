function result = runFormationB4V52RuntimeSmoke()
% RUNFORMATIONB4V52RUNTIMESMOKE Eight-step real-filter integration check.

presetName = 'm24-formation-fov-convoy';
seed = 41;
timeCount = 8;
inputs = generateDynamicTopologyScenarioInputs(presetName, seed);
inputs = truncateInputs(inputs, timeCount);
identity = buildDynamicTopologyPhysicalIdentityRegistry(inputs.config);
[inputs.commConfig.linkUniforms, ~] = ...
    materializePhysicalUidDirectedDeliveryUniforms( ...
        52008, identity.sensorPhysicalUids, timeCount);
model = removeRealizedTargetTruth(inputs.model);
protocol = getFormationB4V52RuntimeProtocol();
triggerConfig = buildFormationB4V52FixedTriggerConfig( ...
    protocol.candidateArmId, inputs.config.numberOfSensors);
context = buildFormationB4V52PairedTrackingDevelopmentContext( ...
    presetName, seed, protocol.candidateArmId);

rng(52009, 'twister');
[~, diagnostics] = runEventTriggeredDistributedLmbFilter( ...
    model, inputs.measurements, inputs.sensorTrajectories, ...
    inputs.neighborMap, inputs.commConfig, triggerConfig, context);

nodeCount = inputs.config.numberOfSensors;
messageCount = reshape(sum(sum( ...
    logical(diagnostics.attempted), 1), 2), 1, []);
pulseExecuted = false(1, timeCount);
pulseForced = false(1, timeCount);
decisionEvaluated = false(1, timeCount);
decisionReasons = repmat({''}, 1, timeCount);
disagreementImprovement = nan(1, timeCount);
cardinalityGain = nan(1, timeCount);
retentionRisk = nan(1, timeCount);
for timeIdx = 1:timeCount
    schedule = diagnostics.topologyPolicyScheduleCertificate{timeIdx};
    pulseExecuted(timeIdx) = schedule.pulseExecuted;
    pulseForced(timeIdx) = schedule.pulseForced;
    decisionEvaluated(timeIdx) = schedule.pulseDecisionEvaluated;
    decisionReasons{timeIdx} = schedule.pulseDecisionReason;
    if schedule.pulseDecisionEvaluated
        decision = schedule.counterfactualDecision;
        disagreementImprovement(timeIdx) = ...
            decision.disagreementImprovementFraction;
        cardinalityGain(timeIdx) = ...
            decision.cardinalityGainFraction;
        retentionRisk(timeIdx) = decision.retentionRisk;
    end
end

windowPulseCounts = zeros(1, timeCount / protocol.period);
windowMessageCounts = zeros(size(windowPulseCounts));
for windowIdx = 1:numel(windowPulseCounts)
    times = (windowIdx - 1) * protocol.period + (1:protocol.period);
    windowPulseCounts(windowIdx) = nnz(pulseExecuted(times));
    windowMessageCounts(windowIdx) = sum(messageCount(times));
end
passed = all(windowPulseCounts == 1) && ...
    all(windowMessageCounts == 5 * nodeCount) && ...
    all(messageCount == nodeCount | messageCount == 2 * nodeCount) && ...
    pulseExecuted(1) && pulseForced(1) && ...
    all(cellfun(@(weights) all(abs(sum(weights, 2) - 1) < 1e-12), ...
        diagnostics.topologyPolicyFusionWeightMatrix));
if ~passed
    error('FormationB4V52Smoke:RuntimeFailed', ...
        'The V52 real-filter runtime smoke failed.');
end

result = struct();
result.presetName = presetName;
result.seed = seed;
result.timeCount = timeCount;
result.messageCountByTime = messageCount;
result.pulseExecutedByTime = pulseExecuted;
result.pulseForcedByTime = pulseForced;
result.decisionEvaluatedByTime = decisionEvaluated;
result.decisionReasonByTime = decisionReasons;
result.disagreementImprovementByTime = disagreementImprovement;
result.cardinalityGainByTime = cardinalityGain;
result.retentionRiskByTime = retentionRisk;
result.windowPulseCounts = windowPulseCounts;
result.windowMessageCounts = windowMessageCounts;
result.totalAttemptedMessageCount = nnz(diagnostics.attempted);
result.totalDeliveredMessageCount = nnz(diagnostics.delivered);
result.trackingOutcomeScored = false;
result.passed = passed;
fprintf(['V52 runtime smoke: messages=%s pulses=%s forced=%s ', ...
    'evaluated=%s reasons=%s\n'], ...
    mat2str(messageCount), mat2str(pulseExecuted), ...
    mat2str(pulseForced), mat2str(decisionEvaluated), ...
    strjoin(decisionReasons, ','));
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
