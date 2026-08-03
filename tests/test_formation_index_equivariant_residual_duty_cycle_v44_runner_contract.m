function test_formation_index_equivariant_residual_duty_cycle_v44_runner_contract()
% Fail-closed runner and executable-manifest boundary checks.

assertErrorId(@() ...
    runFormationIndexEquivariantResidualDutyCycleV44Batch( ...
        [tempname(), '_v44_invalid'], struct( ...
            'presets', {{'m24-formation-fov'}}, 'seeds', 41, ...
            'requireCleanGit', 2)), ...
    'FormationResidualDutyCycleV44Runner:InvalidOptions');
testExistingOutputRejected();
testUntrackedFileRejectedByOfficialRunner();
testPathShadowingRejected();
fprintf('PASS: V44 residual duty-cycle runner-contract tests\n');
end

function testExistingOutputRejected()
outputDirectory = [tempname(), '_v44_existing'];
mkdir(outputDirectory);
cleanup = onCleanup(@() removeIfPresent(outputDirectory)); %#ok<NASGU>
assertErrorId(@() ...
    runFormationIndexEquivariantResidualDutyCycleV44Batch( ...
        outputDirectory, struct( ...
            'presets', {{'m24-formation-fov'}}, 'seeds', 41, ...
            'requireCleanGit', false)), ...
    'FormationResidualDutyCycleV44Runner:OutputExists');
end

function testUntrackedFileRejectedByOfficialRunner()
repoRoot = fileparts(fileparts(mfilename('fullpath')));
sentinelPath = [tempname(repoRoot), '_v44_untracked_sentinel'];
outputDirectory = [tempname(), '_v44_dirty_gate'];
cleanup = onCleanup(@() cleanupDirtyGate( ...
    sentinelPath, outputDirectory)); %#ok<NASGU>
fid = fopen(sentinelPath, 'w');
assert(fid >= 0);
fprintf(fid, 'untracked V44 gate sentinel\n');
fclose(fid);
assertErrorId(@() ...
    runFormationIndexEquivariantResidualDutyCycleV44Batch( ...
        outputDirectory, struct( ...
            'presets', {{'m24-formation-fov'}}, 'seeds', 41, ...
            'requireCleanGit', true)), ...
    'FormationResidualDutyCycleV44Runner:DirtyGitState');
end

function testPathShadowingRejected()
repoRoot = fileparts(fileparts(mfilename('fullpath')));
shadowDirectory = [tempname(), '_v44_shadow'];
mkdir(shadowDirectory);
shadowPath = fullfile(shadowDirectory, ...
    'buildIndexEquivariantResidualDutyCycleSchedule.m');
cleanup = onCleanup(@() cleanupShadow( ...
    shadowDirectory, shadowPath, repoRoot)); %#ok<NASGU>
fid = fopen(shadowPath, 'w');
assert(fid >= 0);
fprintf(fid, ['function varargout = ', ...
    'buildIndexEquivariantResidualDutyCycleSchedule(varargin)\n', ...
    'varargout = cell(1,nargout);\nend\n']);
fclose(fid);
addpath(shadowDirectory, '-begin');
clear buildIndexEquivariantResidualDutyCycleSchedule;
assertErrorId(@() ...
    buildFormationIndexEquivariantResidualDutyCycleV44ExecutableManifest( ...
        repoRoot), ...
    'FormationResidualDutyCycleV44Manifest:PathShadowing');
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
clear buildIndexEquivariantResidualDutyCycleSchedule;
addpath(genpath(repoRoot));
end

function removeIfPresent(path)
if exist(path, 'dir')
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
