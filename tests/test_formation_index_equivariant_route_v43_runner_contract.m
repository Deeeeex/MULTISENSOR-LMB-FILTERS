function test_formation_index_equivariant_route_v43_runner_contract()
testUntrackedFileRejectedByOfficialGate();
testPathShadowingRejected();
outputDirectory = [tempname(), '_v43_runner_contract'];
cleanup = onCleanup(@() removeIfPresent(outputDirectory)); %#ok<NASGU>
result = runFormationIndexEquivariantRouteV43DevelopmentBatch( ...
    outputDirectory, struct( ...
        'presets', {{'m24-formation-fov'}}, ...
        'seeds', 41, 'requireCleanGit', false));
assert(result.caseCount == 1);
assert(~result.registeredBatchComplete);
assert(result.exploratorySubsetOnly);
assert(~result.routeReferenceAdvancePassed);
assert(result.sameMessageBudget);
assert(result.sameMessageAndWeightBudget);
assert(result.registeredInputIdentityPassed);
assert(result.executionBoundaryPassed);
assert(~result.officialCleanSourcePassed);
assert(result.executionSourceStable);
assert(~result.routeReferenceAdvanceCriteriaPassed);
assert(~result.postHocExecutableFreezeAuditPassed);
assert(result.everyCaseFiniteTargetHorizon);
assert(result.everyCaseV43TargetHorizonNotWorse);
assert(result.everyCaseFixedNMinusOneFactorLower);
assert(result.sparseFallbackContractPassed);
assert(~result.anySparseActionStructurallyAuthorized);
assert(result.allSingleFormationSuspensionsFallbackReference);
assert(~result.formalRuntimeObservableBoundaryPassed);
assert(~result.runtimePhysicalUidRegistryIntegrated);
assert(~result.trackingOutcomeScored);
assert(~result.trackingOutcomeAuthorized);
assert(~result.validationClaimAllowed);
assert(result.developmentEvidenceOnly);
assert(~result.posteriorUsed && ~result.truthUsed && ...
    ~result.measurementUsed && ~result.futureOutcomeUsed && ...
    ~result.realizedDeliveryUniformsUsed);
assert(numel(result.canonicalSha256) == 64);
assert(exist(fullfile(outputDirectory, ...
    'FORMATION_INDEX_EQUIVARIANT_ROUTE_V43_DEVELOPMENT_V1.mat'), ...
    'file') == 2);
assert(exist(fullfile(outputDirectory, ...
    'FORMATION_INDEX_EQUIVARIANT_ROUTE_V43_DEVELOPMENT_V1.md'), ...
    'file') == 2);
roundTrip = load(fullfile(outputDirectory, ...
    'FORMATION_INDEX_EQUIVARIANT_ROUTE_V43_DEVELOPMENT_V1.mat'));
assert(isfield(roundTrip, 'result'));
assert(strcmp(roundTrip.result.canonicalSha256, ...
    result.canonicalSha256));
assert(isequaln(roundTrip.result, result));

assertErrorId(@() ...
    runFormationIndexEquivariantRouteV43DevelopmentBatch( ...
        outputDirectory, struct( ...
            'presets', {{'m24-formation-fov'}}, ...
            'seeds', 41, 'requireCleanGit', false)), ...
    'FormationIndexEquivariantV43Runner:OutputExists');
fprintf('PASS: v43 route runner-contract tests\n');
end

function testUntrackedFileRejectedByOfficialGate()
repoRoot = fileparts(fileparts(mfilename('fullpath')));
sentinelPath = [tempname(repoRoot), '_v43_untracked_sentinel'];
outputDirectory = [tempname(), '_v43_dirty_gate'];
cleanup = onCleanup(@() cleanupDirtyGate( ...
    sentinelPath, outputDirectory)); %#ok<NASGU>
fid = fopen(sentinelPath, 'w');
assert(fid >= 0);
fprintf(fid, 'untracked gate sentinel\n');
fclose(fid);
assertErrorId(@() ...
    runFormationIndexEquivariantRouteV43DevelopmentBatch( ...
        outputDirectory, struct( ...
            'presets', {{'m24-formation-fov'}}, ...
            'seeds', 41, 'requireCleanGit', true)), ...
    'FormationIndexEquivariantV43Runner:DirtyGitState');
end

function testPathShadowingRejected()
repoRoot = fileparts(fileparts(mfilename('fullpath')));
shadowDirectory = [tempname(), '_v43_shadow'];
mkdir(shadowDirectory);
shadowPath = fullfile(shadowDirectory, ...
    'selectIndexEquivariantFormationBackbonePolicy.m');
cleanup = onCleanup(@() cleanupShadow( ...
    shadowDirectory, shadowPath, repoRoot)); %#ok<NASGU>
fid = fopen(shadowPath, 'w');
assert(fid >= 0);
fprintf(fid, ['function varargout = ', ...
    'selectIndexEquivariantFormationBackbonePolicy(varargin)\n', ...
    'varargout = cell(1,nargout);\nend\n']);
fclose(fid);
addpath(shadowDirectory, '-begin');
clear selectIndexEquivariantFormationBackbonePolicy;
assertErrorId(@() ...
    buildFormationIndexEquivariantRouteV43ExecutableSourceManifest( ...
        repoRoot), ...
    'FormationIndexEquivariantV43Manifest:PathShadowing');
end

function removeIfPresent(path)
if exist(path, 'dir')
    rmdir(path, 's');
end
end

function cleanupDirtyGate(sentinelPath, outputDirectory)
if exist(sentinelPath, 'file')
    delete(sentinelPath);
end
removeIfPresent(outputDirectory);
end

function cleanupShadow(shadowDirectory, shadowPath, repoRoot)
rmpath(shadowDirectory);
if exist(shadowPath, 'file')
    delete(shadowPath);
end
if exist(shadowDirectory, 'dir')
    rmdir(shadowDirectory, 's');
end
clear selectIndexEquivariantFormationBackbonePolicy;
addpath(genpath(repoRoot));
end

function assertErrorId(callable, expectedId)
thrown = false;
try
    callable();
catch errorInfo
    thrown = true;
    assert(strcmp(errorInfo.identifier, expectedId), ...
        'Unexpected error identifier: %s', errorInfo.identifier);
end
assert(thrown, 'Expected error was not thrown.');
end
