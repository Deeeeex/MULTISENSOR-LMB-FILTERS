function bank = buildSetTrustSequenceCandidatesV134( ...
        rankedFormationIds, formationCycleDiameter, options)
% BUILDSETTRUSTSEQUENCECANDIDATESV134 Binary posterior-admission bank.
%
% Candidate sets are nested prefixes of an observable formation ranking.
% Each set receives either diagnostic persistent abstention or a
% diameter-long abstention phase followed by one-at-a-time binary reentry.
% No intermediate KLA weight is an action: an input is absent (0) or it is
% admitted with its frozen nominal weight (1).

if nargin < 3 || isempty(options)
    options = struct();
end
allowedOptionFields = {'maximumPrefixSize'};
if ~isstruct(options) || ~isscalar(options) || ...
        ~all(ismember(fieldnames(options), allowedOptionFields))
    error('V134SetTrustBank:InvalidOptions', ...
        'Only maximumPrefixSize may configure the binary action bank.');
end
rankedFormationIds = reshape(rankedFormationIds, 1, []);
if ~isnumeric(rankedFormationIds) || isempty(rankedFormationIds) || ...
        any(~isfinite(rankedFormationIds)) || ...
        any(rankedFormationIds < 1) || ...
        any(rankedFormationIds ~= round(rankedFormationIds)) || ...
        numel(unique(rankedFormationIds)) ~= numel(rankedFormationIds) || ...
        ~isscalar(formationCycleDiameter) || ...
        ~isfinite(formationCycleDiameter) || ...
        formationCycleDiameter < 1 || ...
        formationCycleDiameter ~= round(formationCycleDiameter)
    error('V134SetTrustBank:InvalidInput', ...
        'Formation ranks and the directed-cycle diameter are malformed.');
end

maximumPrefixSize = min(numel(rankedFormationIds), round(getField( ...
    options, 'maximumPrefixSize', numel(rankedFormationIds))));
if ~isscalar(maximumPrefixSize) || ~isfinite(maximumPrefixSize) || ...
        maximumPrefixSize < 1
    error('V134SetTrustBank:InvalidMaximumPrefix', ...
        'At least one nested formation prefix is required.');
end

horizonSteps = 2 * formationCycleDiameter + numel(rankedFormationIds);
relativeTimes = 0:(horizonSteps - 1);
emptySchedule = repmat({zeros(1, 0)}, 1, horizonSteps);
actions = makeAction( ...
    'reference-full-trust', 'reference', zeros(1, 0), ...
    emptySchedule, emptySchedule, horizonSteps, false, false);

for prefixSize = 1:maximumPrefixSize
    formationSet = rankedFormationIds(1:prefixSize);
    formationSchedule = repmat({formationSet}, 1, horizonSteps);

    zeroTrust = repmat({zeros(1, prefixSize)}, 1, horizonSteps);
    actions(end+1) = makeAction( ... %#ok<AGROW>
        sprintf('prefix-%02d-persistent-zero', prefixSize), ...
        'persistent-zero', formationSet, formationSchedule, ...
        zeroTrust, horizonSteps, false, true);

    staggeredAdmission = cell(1, horizonSteps);
    for pageIdx = 1:horizonSteps
        if pageIdx <= formationCycleDiameter
            factors = zeros(1, prefixSize);
        else
            restoredCount = min( ...
                pageIdx - formationCycleDiameter, prefixSize);
            factors = zeros(1, prefixSize);
            if restoredCount > 0
                firstRestored = prefixSize - restoredCount + 1;
                factors(firstRestored:prefixSize) = 1;
            end
        end
        staggeredAdmission{pageIdx} = factors;
    end
    actions(end+1) = makeAction( ... %#ok<AGROW>
        sprintf('prefix-%02d-staggered-binary-reentry', prefixSize), ...
        'staggered-binary-reentry', formationSet, formationSchedule, ...
        staggeredAdmission, horizonSteps, true, false);
end

bank = struct();
bank.contractVersion = 'v134-binary-admission-sequence-bank-v2';
bank.referenceActionIndex = 1;
bank.rankedFormationIds = rankedFormationIds;
bank.formationCycleDiameter = formationCycleDiameter;
bank.admissionFactors = [0, 1];
bank.horizonSteps = horizonSteps;
bank.relativeTimes = relativeTimes;
bank.actions = actions;
bank.actionCount = numel(actions);
bank.maximumPrefixSize = maximumPrefixSize;
bank.fullPosteriorPreservedWheneverTrustPositive = true;
bank.zeroTrustUsesControlSynopsisOnly = true;
bank.intermediateTrustAllowed = false;
bank.learnedFusionWeightAllocationAllowed = false;
end

function action = makeAction( ...
        name, scheduleKind, formationSet, formationsByTime, ...
        trustFactorsByTime, horizonSteps, gateEligible, diagnosticOnly)
action = struct();
action.name = name;
action.scheduleKind = scheduleKind;
action.formationSet = reshape(formationSet, 1, []);
action.formationIdsByTime = formationsByTime;
action.trustFactorsByTime = trustFactorsByTime;
action.gateEligible = logical(gateEligible);
action.diagnosticOnly = logical(diagnosticOnly);
action.zeroTrustPageCount = 0;
action.fullPayloadPageCount = 0;
for pageIdx = 1:horizonSteps
    factors = reshape(trustFactorsByTime{pageIdx}, 1, []);
    action.zeroTrustPageCount = action.zeroTrustPageCount + ...
        nnz(factors <= 1e-12);
    action.fullPayloadPageCount = action.fullPayloadPageCount + ...
        nnz(factors > 1e-12);
end
action.returnsToFullTrust = isempty(action.formationSet) || ...
    all(trustFactorsByTime{end} >= 1 - 1e-12);
end

function value = getField(structure, fieldName, defaultValue)
if isstruct(structure) && isfield(structure, fieldName)
    value = structure.(fieldName);
else
    value = defaultValue;
end
end
