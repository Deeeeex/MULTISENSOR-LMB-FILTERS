function projection = ...
        projectBudgetedObservationSupportedRepairV197( ...
            setSafeProjection, previousReleaseHistory, protocol)
% PROJECTBUDGETEDOBSERVATIONSUPPORTEDREPAIRV197 Spend one repair token.

if nargin < 3 || isempty(protocol)
    protocol = getBudgetedObservationSupportedRepairV197Protocol();
end
required = { ...
    'groups', 'proposedFormationIds', 'releaseFormationIds', ...
    'maximumReceiverRiskByFormation'};
if ~isstruct(setSafeProjection) || ~isscalar(setSafeProjection) || ...
        ~all(isfield(setSafeProjection, required)) || ...
        ~iscell(previousReleaseHistory) || ...
        numel(previousReleaseHistory) > protocol.releaseHistoryDepth
    error('BudgetedSetRepairV197:InvalidInput', ...
        'The set-safe projection or release history is invalid.');
end
groups = reshape(setSafeProjection.groups, 1, []);
proposed = reshape( ...
    setSafeProjection.proposedFormationIds, 1, []);
risky = reshape(setSafeProjection.releaseFormationIds, 1, []);
for pageIdx = 1:numel(previousReleaseHistory)
    releases = reshape(previousReleaseHistory{pageIdx}, 1, []);
    if ~isnumeric(releases) || any(~isfinite(releases)) || ...
            any(~ismember(releases, groups)) || ...
            numel(unique(releases)) ~= numel(releases)
        error('BudgetedSetRepairV197:InvalidInput', ...
            'A past release page is malformed.');
    end
end
recentReleaseCount = sum(cellfun( ...
    @(ids) numel(ids), previousReleaseHistory));
tokenAvailable = recentReleaseCount == 0;
selectedRelease = zeros(1, 0);
selectedRisk = NaN;
if tokenAvailable && ~isempty(risky)
    riskyIdx = zeros(1, numel(risky));
    for idx = 1:numel(risky)
        riskyIdx(idx) = find(groups == risky(idx), 1);
    end
    risks = reshape( ...
        setSafeProjection.maximumReceiverRiskByFormation, 1, []);
    if numel(risks) ~= numel(groups) || ...
            any(~isfinite(risks(riskyIdx)))
        error('BudgetedSetRepairV197:InvalidRisk', ...
            'A releasable formation lacks finite set-entry risk.');
    end
    [selectedRisk, localIdx] = max(risks(riskyIdx));
    selectedRelease = risky(localIdx);
end

projection = setSafeProjection;
projection.contractVersion = ...
    'budgeted-observation-supported-repair-v197-projection-v1';
projection.unbudgetedReleaseFormationIds = risky;
projection.releaseFormationIds = selectedRelease;
projection.selectedFormationIds = setdiff( ...
    proposed, selectedRelease, 'stable');
projection.releaseTokenAvailable = tokenAvailable;
projection.recentReleaseCount = recentReleaseCount;
projection.releaseHistoryDepth = protocol.releaseHistoryDepth;
projection.releaseCooldownSteps = protocol.releaseCooldownSteps;
projection.maximumReleaseFormationsPerWindow = ...
    protocol.maximumReleaseFormationsPerWindow;
projection.selectedReleaseRisk = selectedRisk;
projection.releaseBudgetApplied = ...
    ~isequal(selectedRelease, risky);
projection.truthUsed = false;
projection.futureInformationUsed = false;
end

