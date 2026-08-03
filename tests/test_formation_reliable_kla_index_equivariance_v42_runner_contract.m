function test_formation_reliable_kla_index_equivariance_v42_runner_contract()
outputDirectory = [tempname(), '_v42_runner_contract'];
cleanup = onCleanup(@() removeOutput(outputDirectory)); %#ok<NASGU>
options = struct( ...
    'presets', {{'m24-formation-fov'}}, ...
    'seeds', 41, 'requireCleanGit', false);
result = ...
    runFormationReliableKlaIndexEquivarianceV42DevelopmentProbe( ...
        outputDirectory, options);

assert(result.caseCount == 1);
assert(~result.registeredBatchComplete);
assert(result.exploratorySubsetOnly);
assert(numel(result.requestedCaseIds) == 1);
assert(numel(result.registeredCaseIds) == 8);
assert(~result.registeredBatchTestedCyclicBlockOrderEquivariant);
assert(~result.fullFormationBlockPermutationFamilyCovered);
assert(~result.withinFormationSensorPermutationCovered);
assert(~result.physicalActionInterpretationAuthorized);
assert(result.compactReconstructableArtifact == ...
    ~result.generationGitDirty);
assert(result.sourceIdentityCapturedByCleanGitCommit == ...
    ~result.generationGitDirty);
assert(numel(result.executableSourceManifest.canonicalSha256) == 64);
assert(strcmp(result.canonicalSha256, ...
    computeCanonicalValueSha256(rmfield(result, 'canonicalSha256'))));

current = result.cases(1);
assert(numel(current.shifts) == current.formationCount);
for shiftIdx = 1:numel(current.shifts)
    shift = current.shifts(shiftIdx);
    assert(isequal(shift.candidatePhysicalFormationIds, ...
        current.physicalFormationIds));
    assert(numel(shift.newToOldNodeOrder) == current.nodeCount);
    assert(numel(unique(shift.newToOldNodeOrder)) == current.nodeCount);
end

matPath = fullfile(outputDirectory, ...
    'FORMATION_RELIABLE_KLA_INDEX_EQUIVARIANCE_V42_DEVELOPMENT_V1.mat');
reportPath = fullfile(outputDirectory, ...
    'FORMATION_RELIABLE_KLA_INDEX_EQUIVARIANCE_V42_DEVELOPMENT_V1.md');
assert(exist(matPath, 'file') == 2);
assert(exist(reportPath, 'file') == 2);
stored = load(matPath).result;
assert(strcmp(stored.canonicalSha256, result.canonicalSha256));
report = fileread(reportPath);
assert(~isempty(strfind(report, ...
    'exploratory subset only: `1`'))); %#ok<STREMP>
assert(~isempty(strfind(report, ...
    'physicalActionInterpretationAuthorized=false'))); %#ok<STREMP>

assertErrorId(@() ...
    runFormationReliableKlaIndexEquivarianceV42DevelopmentProbe( ...
        outputDirectory, options), ...
    'FormationKlaIndexEquivarianceRunner:OutputExists');
fprintf('Formation reliable-KLA v42 runner-contract tests passed.\n');
end

function removeOutput(path)
if exist(path, 'dir') == 7
    rmdir(path, 's');
end
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
