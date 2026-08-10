function context = ...
    buildNetworkBudgetReallocatedMultiSourceV95ExecutionContext( ...
        presetName, seed, currentTime, actionName, actionSequenceIndices)
% BUILDNETWORKBUDGETREALLOCATEDMULTISOURCEV95EXECUTIONCONTEXT Paired arms.

protocol = getNetworkBudgetReallocatedMultiSourceV95Protocol();
caseInfo = resolveCase(presetName, seed, currentTime, protocol);
sequence = reshape(actionSequenceIndices, 1, []);
if isempty(caseInfo)
    error('NetworkBudgetV95:InvalidExecutionContext', ...
        'The V95 request is not a registered cross-scale anchor.');
end
validSequences = buildRegisteredSequences(caseInfo.horizonSteps);
if ~ischar(actionName) || isempty(actionName) || ...
        numel(sequence) ~= caseInfo.horizonSteps || ...
        ~any(all(bsxfun(@eq, validSequences, sequence), 2))
    error('NetworkBudgetV95:InvalidExecutionContext', ...
        'The V95 request is not one of its four frozen paired arms.');
end

context = struct();
context.contractVersion = ...
    'network-budget-reallocated-multi-source-v95-context-v1';
context.capability = ...
    'network-budget-reallocated-multi-source-v95-development';
context.action = ...
    'filter-network-budget-reallocated-multi-source-v95-development';
context.protocolId = protocol.id;
context.phase = 'opened-return';
context.presetName = presetName;
context.seed = seed;
context.currentTime = currentTime;
context.measurementTimeCount = ...
    currentTime + caseInfo.horizonSteps - 1;
context.policyName = protocol.outcomePolicyName;
context.actionName = actionName;
context.actionSequenceIndices = sequence;
context.developmentEvidenceOnly = true;
end

function caseInfo = resolveCase(presetName, seed, currentTime, protocol)
caseInfo = [];
for candidate = protocol.cases
    if strcmp(presetName, candidate.presetName) && ...
            seed == candidate.seed && ...
            currentTime == candidate.currentTime
        caseInfo = candidate;
        return;
    end
end
end

function sequences = buildRegisteredSequences(horizonSteps)
reference = ones(1, horizonSteps);
donorOnly = reference;
candidate = reference;
donorOnly(1) = 2;
candidate(1) = 3;
fixedCandidate = 3 * ones(1, horizonSteps);
sequences = [reference; donorOnly; candidate; fixedCandidate];
end
