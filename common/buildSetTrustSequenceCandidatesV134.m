function bank = buildSetTrustSequenceCandidatesV134( ...
        rankedFormationIds, formationCycleDiameter, options)
% BUILDSETTRUSTSEQUENCECANDIDATESV134 Compact finite-horizon action bank.
%
% Candidate sets are nested prefixes of an observable formation ranking.
% Each set receives either persistent zero trust or a diameter-long zero
% trust phase followed by a registered gradual return.  This keeps the bank
% linear in formation count and makes the propagation time scale-aware.

if nargin < 3 || isempty(options)
    options = struct();
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

returnRamp = reshape(getField(options, ...
    'returnRamp', [0.25, 0.50, 1.00]), 1, []);
if ~isnumeric(returnRamp) || isempty(returnRamp) || ...
        any(~isfinite(returnRamp)) || any(returnRamp <= 0) || ...
        any(returnRamp > 1) || any(diff(returnRamp) <= 0) || ...
        abs(returnRamp(end) - 1) > 1e-12
    error('V134SetTrustBank:InvalidReturnRamp', ...
        'The trust return ramp must increase strictly to one.');
end
maximumPrefixSize = min(numel(rankedFormationIds), round(getField( ...
    options, 'maximumPrefixSize', numel(rankedFormationIds))));
if ~isscalar(maximumPrefixSize) || ~isfinite(maximumPrefixSize) || ...
        maximumPrefixSize < 1
    error('V134SetTrustBank:InvalidMaximumPrefix', ...
        'At least one nested formation prefix is required.');
end

horizonSteps = formationCycleDiameter + numel(returnRamp);
relativeTimes = 0:(horizonSteps - 1);
emptySchedule = repmat({zeros(1, 0)}, 1, horizonSteps);
actions = makeAction( ...
    'reference-full-trust', 'reference', zeros(1, 0), ...
    emptySchedule, emptySchedule, horizonSteps);

for prefixSize = 1:maximumPrefixSize
    formationSet = rankedFormationIds(1:prefixSize);
    formationSchedule = repmat({formationSet}, 1, horizonSteps);

    zeroTrust = repmat({zeros(1, prefixSize)}, 1, horizonSteps);
    actions(end+1) = makeAction( ... %#ok<AGROW>
        sprintf('prefix-%02d-persistent-zero', prefixSize), ...
        'persistent-zero', formationSet, formationSchedule, ...
        zeroTrust, horizonSteps);

    taperedTrust = cell(1, horizonSteps);
    for pageIdx = 1:horizonSteps
        if pageIdx <= formationCycleDiameter
            factor = 0;
        else
            factor = returnRamp(pageIdx - formationCycleDiameter);
        end
        taperedTrust{pageIdx} = factor * ones(1, prefixSize);
    end
    actions(end+1) = makeAction( ... %#ok<AGROW>
        sprintf('prefix-%02d-diameter-taper', prefixSize), ...
        'diameter-taper', formationSet, formationSchedule, ...
        taperedTrust, horizonSteps);
end

bank = struct();
bank.contractVersion = 'v134-set-trust-sequence-bank-v1';
bank.referenceActionIndex = 1;
bank.rankedFormationIds = rankedFormationIds;
bank.formationCycleDiameter = formationCycleDiameter;
bank.returnRamp = returnRamp;
bank.horizonSteps = horizonSteps;
bank.relativeTimes = relativeTimes;
bank.actions = actions;
bank.actionCount = numel(actions);
bank.maximumPrefixSize = maximumPrefixSize;
bank.fullPosteriorPreservedWheneverTrustPositive = true;
bank.zeroTrustUsesControlSynopsisOnly = true;
end

function action = makeAction( ...
        name, scheduleKind, formationSet, formationsByTime, ...
        trustFactorsByTime, horizonSteps)
action = struct();
action.name = name;
action.scheduleKind = scheduleKind;
action.formationSet = reshape(formationSet, 1, []);
action.formationIdsByTime = formationsByTime;
action.trustFactorsByTime = trustFactorsByTime;
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
