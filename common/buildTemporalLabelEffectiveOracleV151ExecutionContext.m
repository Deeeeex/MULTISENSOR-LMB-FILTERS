function context = ...
    buildTemporalLabelEffectiveOracleV151ExecutionContext( ...
        presetName, seed, currentTime, actionName, omissionEnabled)
% BUILDTEMPORALLABELEFFECTIVEORACLEV151EXECUTIONCONTEXT Repeated H=8.

protocol = getTemporalLabelEffectiveOracleV151Protocol();
caseEntry = resolveCase(protocol, presetName, seed, currentTime);
if ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(omissionEnabled) || ...
        ~(islogical(omissionEnabled) || isnumeric(omissionEnabled))
    error('TemporalLabelOracleV151:InvalidContextInput', ...
        'The V151 action name or omission flag is invalid.');
end
context = struct();
context.contractVersion = ...
    'temporal-label-effective-oracle-v151-context-v1';
context.capability = 'temporal-label-effective-oracle-development';
context.action = 'filter-temporal-label-effective-oracle-development';
context.protocolId = protocol.id;
context.phase = 'opened-return';
context.presetName = caseEntry.presetName;
context.seed = caseEntry.seed;
context.currentTime = caseEntry.currentTime;
context.measurementTimeCount = ...
    caseEntry.currentTime + caseEntry.horizonSteps - 1;
context.policyName = protocol.policyName;
context.actionName = actionName;
context.explicitLabelOmissionEnabled = logical(omissionEnabled);
context.developmentEvidenceOnly = true;
end

function entry = resolveCase(protocol, presetName, seed, currentTime)
matches = strcmp({protocol.cases.presetName}, presetName) & ...
    [protocol.cases.seed] == seed & ...
    [protocol.cases.currentTime] == currentTime;
if nnz(matches) ~= 1
    error('TemporalLabelOracleV151:UnregisteredCase', ...
        'The requested V151 case is not registered.');
end
entry = protocol.cases(matches);
end
