function context = buildCounterfactualRegretGateV133ExecutionContext( ...
        presetName, seed, evidenceSplit, phase, currentTime, ...
        referenceCarrierMode, protectedFormationId)
% BUILDCOUNTERFACTUALREGRETGATEV133EXECUTIONCONTEXT Teacher-only permit.

% The permit covers a causal reference prefix or one paired development /
% calibration continuation.  It deliberately excludes unseen-validation
% seeds until the model and operating threshold have been frozen.

protocol = getCounterfactualRegretGateV133Protocol();
evidenceSplit = lower(strrep(char(evidenceSplit), '_', '-'));
phase = lower(strrep(char(phase), '_', '-'));
referenceCarrierMode = lower(strrep( ...
    char(referenceCarrierMode), '_', '-'));
if ~ischar(presetName) || isempty(presetName) || ...
        ~isscalar(seed) || ~isfinite(seed) || seed ~= round(seed) || ...
        ~ismember(evidenceSplit, {'development', 'calibration'}) || ...
        ~ismember(phase, { ...
            'baseline-selection', 'reference-prefix', ...
            'reference-continuation', ...
            'candidate-continuation'}) || ...
        ~isscalar(currentTime) || ~isfinite(currentTime) || ...
        currentTime < 1 || currentTime ~= round(currentTime) || ...
        ~ismember(referenceCarrierMode, ...
            protocol.referenceCarrierModes) || ...
        ~isscalar(protectedFormationId) || ...
        ~isfinite(protectedFormationId) || ...
        protectedFormationId < 0 || ...
        protectedFormationId ~= round(protectedFormationId)
    error('CounterfactualRegretV133:InvalidExecutionContext', ...
        'The V133 teacher execution-context input is invalid.');
end
caseInfo = resolveCase(protocol, presetName);
if strcmp(evidenceSplit, 'development')
    allowedSeeds = protocol.developmentSeeds;
else
    allowedSeeds = protocol.calibrationSeeds;
end
if ~ismember(seed, allowedSeeds)
    error('CounterfactualRegretV133:InvalidExecutionContext', ...
        'The seed is outside the requested trajectory-level split.');
end
anchorTimes = resolveAnchors(caseInfo.presetName, protocol);
if strcmp(phase, 'baseline-selection')
    scenarioConfig = buildDynamicTopologyScenarioConfig(presetName);
    if currentTime ~= scenarioConfig.simulationLength || ...
            protectedFormationId ~= 0
        error('CounterfactualRegretV133:InvalidExecutionContext', ...
            'Baseline selection must cover the complete trajectory.');
    end
    horizonSteps = 1;
    measurementTimeCount = currentTime;
    scheduleEnabled = false;
    actionName = ['baseline-', referenceCarrierMode];
elseif strcmp(phase, 'reference-prefix')
    if currentTime ~= max(anchorTimes) || protectedFormationId ~= 0
        error('CounterfactualRegretV133:InvalidExecutionContext', ...
            'The prefix permit must end at the last registered anchor.');
    end
    horizonSteps = 1;
    measurementTimeCount = currentTime;
    scheduleEnabled = false;
    actionName = 'reference-prefix';
elseif strcmp(phase, 'reference-continuation')
    if ~ismember(currentTime, anchorTimes) || protectedFormationId ~= 0
        error('CounterfactualRegretV133:InvalidExecutionContext', ...
            'The reference continuation must use a registered anchor.');
    end
    horizonSteps = caseInfo.horizonSteps;
    measurementTimeCount = currentTime + horizonSteps - 1;
    scheduleEnabled = false;
    actionName = 'reference-full-posterior';
else
    if ~ismember(currentTime, anchorTimes) || ...
            protectedFormationId < 1 || ...
            protectedFormationId > caseInfo.expectedFormationCount
        error('CounterfactualRegretV133:InvalidExecutionContext', ...
            'The candidate continuation action is outside the contract.');
    end
    horizonSteps = caseInfo.horizonSteps;
    measurementTimeCount = currentTime + horizonSteps - 1;
    scheduleEnabled = true;
    actionName = sprintf( ...
        'abstain-current-page-receiver-formation-%d', ...
        protectedFormationId);
end

context = struct();
context.contractVersion = ...
    'counterfactual-regret-gate-v133-execution-context-v1';
context.capability = ...
    'counterfactual-regret-gate-v133-teacher-generation';
context.action = ...
    'filter-counterfactual-regret-gate-v133-teacher-generation';
context.protocolId = protocol.id;
context.evidenceSplit = evidenceSplit;
context.phase = phase;
context.presetName = presetName;
context.seed = seed;
context.currentTime = currentTime;
context.horizonSteps = horizonSteps;
context.measurementTimeCount = measurementTimeCount;
context.policyName = protocol.outcomePolicyName;
context.actionName = actionName;
context.referenceCarrierMode = referenceCarrierMode;
context.protectedFormationId = protectedFormationId;
context.scheduleEnabled = logical(scheduleEnabled);
context.onlineReselectionEnabled = false;
context.developmentEvidenceOnly = true;
end

function caseInfo = resolveCase(protocol, presetName)
caseInfo = struct([]);
for candidate = protocol.scaleCases
    if strcmp(candidate.presetName, presetName)
        caseInfo = candidate;
        return;
    end
end
error('CounterfactualRegretV133:InvalidExecutionContext', ...
    'The preset is outside the V133 scale contract.');
end

function anchors = resolveAnchors(presetName, protocol)
config = buildDynamicTopologyScenarioConfig(presetName);
window = reshape(config.focusWindow, 1, []);
anchors = round(window(1) + ...
    protocol.anchorQuantilesWithinFocusWindow * ...
        (window(2) - window(1)));
end
