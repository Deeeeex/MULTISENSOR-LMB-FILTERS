function test_formation_b4_v46_long_horizon_mixing_audit()
short = runFormationB4V46LongHorizonMixingAudit(struct( ...
    'presets', {{'m24-formation-fov'}}, 'seeds', 41, ...
    'timeIndices', 1:12, 'horizons', [4, 8, 12], ...
    'progressEvery', 12));
assert(short.caseCount == 1 && ~short.fullRegistryRequested);
assert(~short.allCasesFullHorizonCovered);
assert(short.allParentPageHashesMatched);
assert(short.everyRollingB4UnionStrong);
assert(~short.allTheoremHorizonsPresent);
assert(~short.longHorizonConvergenceCertificatePassed);
item = short.cases(1);
assert(item.parentPageHashesMatched && item.repairPageCount == 0);
assert(abs(item.minimumPositiveExecutedWeight - 0.1) < 1e-12);
assert(isequal(item.horizons, [4, 8, 12]));
assert(numel(item.horizonSummaries) == 3);
assert(item.horizonSummaries(1).horizon == 4);
assert(item.horizonSummaries(1).windowCount == 9);
assert(abs(item.horizonSummaries(1).worstDobrushin - 1) < 1e-12);
directChecks = ...
    [item.horizonSummaries.productTreeDirectComparisonPassed];
assert(all(directChecks));
assert(~item.trackingOutcomeScored && ...
    ~item.trackingOutcomeAuthorized && ...
    ~item.validationClaimAllowed && item.developmentEvidenceOnly);

crossing = runFormationB4V46LongHorizonMixingAudit(struct( ...
    'presets', {{'x36-formation-fov-crossing'}}, 'seeds', 41, ...
    'timeIndices', 157:160, 'horizons', 4, 'progressEvery', 4));
crossingCase = crossing.cases(1);
assert(crossing.allParentPageHashesMatched);
assert(isequal(crossingCase.repairCountByTime, [0, 1, 2, 3]));
assert(crossingCase.repairPageCount == 3);
assert(crossingCase.horizonSummaries(1).windowCount == 1);
assert(abs(crossingCase.horizonSummaries(1).worstDobrushin - 1) ...
    < 1e-12);
assert(numel(short.canonicalSha256) == 64 && ...
    numel(crossing.canonicalSha256) == 64);
expectInvalidHorizon();
fprintf('PASS: V46 long-horizon mixing audit tests\n');
end

function expectInvalidHorizon()
failed = false;
try
    runFormationB4V46LongHorizonMixingAudit(struct( ...
        'presets', {{'m24-formation-fov'}}, 'seeds', 41, ...
        'timeIndices', 1:4, 'horizons', 8));
catch errorInfo
    failed = strcmp(errorInfo.identifier, ...
        'FormationB4V46LongMixing:InvalidOptions');
end
assert(failed);
end
