function context = ...
    buildOutputAlignedLabelEffectiveOracleV150ExecutionContext( ...
        presetName, seed, currentTime, actionName, omissionEnabled)
% BUILDOUTPUTALIGNEDLABELEFFECTIVEORACLEV150EXECUTIONCONTEXT H=8 only.

protocol = getOutputAlignedLabelEffectiveOracleV150Protocol();
caseEntry = resolveCase(protocol, presetName, seed, currentTime);
if ~ischar(actionName) || isempty(actionName) || ...
        ~isscalar(omissionEnabled) || ...
        ~(islogical(omissionEnabled) || isnumeric(omissionEnabled))
    error('OutputAlignedLabelOracleV150:InvalidContextInput', ...
        'The V150 action name or omission flag is invalid.');
end
context = struct();
context.contractVersion = ...
    'output-aligned-label-effective-oracle-v150-context-v1';
context.capability = ...
    'output-aligned-label-effective-oracle-development';
context.action = ...
    'filter-output-aligned-label-effective-oracle-development';
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
    error('OutputAlignedLabelOracleV150:UnregisteredCase', ...
        'The requested V150 case is not registered.');
end
entry = protocol.cases(matches);
end
