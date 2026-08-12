function bank = buildSetTrustSequenceCandidatesV134( ...
        rankedFormationIds, formationCycleDiameter, ...
        recoveryTailSteps, options)
% BUILDSETTRUSTSEQUENCECANDIDATESV134 Binary posterior-admission bank.
%
% Candidate sets cover every singleton and pair in observable rank space.
% Larger sets remain nested prefixes to keep the bank quadratic rather than
% exponential. Each set receives either diagnostic persistent abstention or
% a diameter-long abstention phase followed by one-at-a-time binary reentry.
% No intermediate KLA weight is an action: an input is absent (0) or it is
% admitted with its frozen nominal weight (1).

if nargin < 4 || isempty(options)
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
        formationCycleDiameter ~= round(formationCycleDiameter) || ...
        ~isscalar(recoveryTailSteps) || ...
        ~isfinite(recoveryTailSteps) || recoveryTailSteps < 1 || ...
        recoveryTailSteps ~= round(recoveryTailSteps)
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

horizonSteps = formationCycleDiameter + ...
    numel(rankedFormationIds) + recoveryTailSteps;
relativeTimes = 0:(horizonSteps - 1);
emptySchedule = repmat({zeros(1, 0)}, 1, horizonSteps);
actions = makeAction( ...
    'reference-full-trust', 'reference', zeros(1, 0), ...
    emptySchedule, emptySchedule, horizonSteps, false, false);

candidateSets = buildCandidateSets( ...
    rankedFormationIds, maximumPrefixSize);
for setIdx = 1:numel(candidateSets)
    candidate = candidateSets(setIdx);
    formationSet = candidate.formationSet;
    setName = candidate.name;
    formationSchedule = repmat({formationSet}, 1, horizonSteps);

    setSize = numel(formationSet);
    zeroTrust = repmat({zeros(1, setSize)}, 1, horizonSteps);
    actions(end+1) = makeAction( ... %#ok<AGROW>
        sprintf('%s-persistent-zero', setName), ...
        'persistent-zero', formationSet, formationSchedule, ...
        zeroTrust, horizonSteps, false, true);

    staggeredAdmission = cell(1, horizonSteps);
    for pageIdx = 1:horizonSteps
        if pageIdx <= formationCycleDiameter
            factors = zeros(1, setSize);
        else
            restoredCount = min( ...
                pageIdx - formationCycleDiameter, setSize);
            factors = zeros(1, setSize);
            if restoredCount > 0
                firstRestored = setSize - restoredCount + 1;
                factors(firstRestored:setSize) = 1;
            end
        end
        staggeredAdmission{pageIdx} = factors;
    end
    actions(end+1) = makeAction( ... %#ok<AGROW>
        sprintf('%s-staggered-binary-reentry', setName), ...
        'staggered-binary-reentry', formationSet, formationSchedule, ...
        staggeredAdmission, horizonSteps, true, false);
end

bank = struct();
bank.contractVersion = 'v134-binary-admission-sequence-bank-v4';
bank.referenceActionIndex = 1;
bank.rankedFormationIds = rankedFormationIds;
bank.formationCycleDiameter = formationCycleDiameter;
bank.recoveryTailSteps = recoveryTailSteps;
bank.admissionFactors = [0, 1];
bank.horizonSteps = horizonSteps;
bank.relativeTimes = relativeTimes;
bank.actions = actions;
bank.actionCount = numel(actions);
bank.deployableSetCount = numel(candidateSets);
bank.maximumPrefixSize = maximumPrefixSize;
bank.candidateSetFamily = ...
    'rank-equivariant-all-singletons-all-pairs-high-order-prefixes';
bank.allSingletonsCovered = true;
bank.allPairsCovered = numel(rankedFormationIds) >= 2;
bank.highOrderSetsAreNestedPrefixes = true;
bank.exponentialSubsetEnumerationUsed = false;
bank.fullPosteriorPreservedWheneverTrustPositive = true;
bank.zeroTrustUsesControlSynopsisOnly = true;
bank.intermediateTrustAllowed = false;
bank.learnedFusionWeightAllocationAllowed = false;
end

function sets = buildCandidateSets(rankedFormationIds, maximumPrefixSize)
formationCount = numel(rankedFormationIds);
sets = repmat(struct('name', '', 'formationSet', zeros(1, 0)), ...
    1, 0);
for rankIdx = 1:formationCount
    sets(end+1) = struct( ... %#ok<AGROW>
        'name', sprintf('singleton-rank-%02d', rankIdx), ...
        'formationSet', rankedFormationIds(rankIdx));
end
if formationCount >= 2
    rankPairs = nchoosek(1:formationCount, 2);
    for pairIdx = 1:size(rankPairs, 1)
        ranks = rankPairs(pairIdx, :);
        sets(end+1) = struct( ... %#ok<AGROW>
            'name', sprintf('pair-ranks-%02d-%02d', ...
                ranks(1), ranks(2)), ...
            'formationSet', rankedFormationIds(ranks));
    end
end
for prefixSize = 3:maximumPrefixSize
    sets(end+1) = struct( ... %#ok<AGROW>
        'name', sprintf('prefix-ranks-01-%02d', prefixSize), ...
        'formationSet', rankedFormationIds(1:prefixSize));
end
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
