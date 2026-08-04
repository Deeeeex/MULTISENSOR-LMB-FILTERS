function test_formation_b4_v46_development_execution_surface_manifest()
% Frozen surface covers executable code, excludes tests, and rejects shadowing.

manifest = buildFormationB4V46DevelopmentExecutionSurfaceManifest();
assert(strcmp(manifest.contractVersion, ...
    'formation-b4-v46-development-execution-surface-manifest-v1'));
assert(strcmp(manifest.exclusionContractVersion, ...
    'tests-only-excluded-v1'));
assert(isempty(manifest.untrackedExecutablePaths));
assert(numel(manifest.paths) == numel(manifest.sha256));
assert(~isempty(manifest.paths));
assert(all(cellfun(@(value) ~isempty(regexp(value, ...
    '^[0-9a-f]{64}$', 'once')), manifest.sha256)));
assert(any(strcmp(manifest.paths, ...
    'common/validateFormationB4V46DevelopmentArmExecution.m')));
assert(any(strcmp(manifest.paths, ...
    'RUN/GA/runFormationB4V46DevelopmentArmShard.m')));
assert(any(strcmp(manifest.paths, ...
    'RUN/GA/launchFormationB4V46DevelopmentArmShard.sh')));
assert(any(strcmp(manifest.paths, ...
    'RUN/GA/private/buildFormationB4V46DevelopmentArmShardPrivate.m')));
assert(any(strcmp(manifest.paths, ...
    'multisensorLmb/runEventTriggeredDistributedLmbFilter.m')));
assert(~any(strncmp(manifest.paths, 'tests/', numel('tests/'))));
assert(~any(strcmp(manifest.paths, ...
    'contracts/formation_b4_v46_development_arm_permits.json')));
assert(strcmp(computeCanonicalValueSha256( ...
    rmfield(manifest, 'canonicalSha256')), manifest.canonicalSha256));

resolution = assertFormationB4V46DevelopmentCriticalFunctionPaths();
assert(numel(resolution.functionNames) == numel(resolution.resolvedPaths));
assert(numel(resolution.functionNames) == ...
    numel(resolution.expectedPathGroups));
assert(numel(resolution.relativePaths) >= numel(resolution.functionNames));
inactive = ismember(resolution.functionNames, ...
    resolution.inactivePlatformVariantFunctionNames);
assert(all(cellfun(@(value) ~isempty(value), ...
    resolution.resolvedPaths(~inactive))));
assert(all(cellfun(@isempty, resolution.resolvedPaths(inactive))));
assert(numel(resolution.functionNames) > 600);
assert(resolution.allTrackedNonTestExecutablesCovered);
assert(resolution.externalPathShadowingRejected);
assert(resolution.privateExecutablesBoundByCallerDirectory);
assert(resolution.gitWorktreePointerAnchored);
assert(resolution.gitEnvironmentOverridesRejected);
assert(any(strcmp(resolution.privateScopedPaths, ...
    'RUN/GA/private/buildFormationB4V46DevelopmentArmShardPrivate.m')));
assert(~any(strcmp(resolution.functionNames, ...
    'buildFormationB4V46DevelopmentArmShardPrivate')));
assert(any(strcmp(resolution.functionNames, 'computeTextSha256')));
assert(any(strcmp(resolution.functionNames, 'computeFileSha256')));
assert(any(strcmp(resolution.functionNames, 'lmbPredictionStep')));
assert(any(strcmp(resolution.functionNames, ...
    'buildFormationB4V46FixedRuntimeArm')));
assert(any(strcmp(resolution.functionNames, ...
    'assertFormationB4V46DevelopmentPermitActivationFileAnchored')));

shadowDirectory = tempname();
mkdir(shadowDirectory);
shadowPath = fullfile(shadowDirectory, 'computeTextSha256.m');
fid = fopen(shadowPath, 'w');
assert(fid >= 0);
fprintf(fid, ['function value = computeTextSha256(varargin)\n', ...
    'value = repmat(''0'', 1, 64);\n', ...
    'end\n']);
fclose(fid);
addpath(shadowDirectory, '-begin');
clear computeTextSha256;
cleanup = onCleanup(@() cleanupShadow(shadowDirectory, shadowPath));
assertErrorId(@() ...
    assertFormationB4V46DevelopmentCriticalFunctionPaths(), ...
    'FormationB4V46DevelopmentPermit:FunctionPathShadowed');
clear cleanup;

checkerShadowDirectory = tempname();
mkdir(checkerShadowDirectory);
checkerShadowPath = fullfile(checkerShadowDirectory, ...
    'assertFormationB4V46DevelopmentCriticalFunctionPaths.m');
fid = fopen(checkerShadowPath, 'w');
assert(fid >= 0);
fprintf(fid, ['function value = ', ...
    'assertFormationB4V46DevelopmentCriticalFunctionPaths(varargin)\n', ...
    'value = struct();\n', ...
    'end\n']);
fclose(fid);
addpath(checkerShadowDirectory, '-begin');
clear assertFormationB4V46DevelopmentCriticalFunctionPaths;
checkerCleanup = onCleanup(@() cleanupCheckerShadow( ...
    checkerShadowDirectory, checkerShadowPath));
assertErrorId(@() runFormationB4V46DevelopmentArmShard( ...
    1, 'v46-repaired-reference-a70-e05'), ...
    'FormationB4V46DevelopmentPermit:BootstrapPathShadowed');
clear checkerCleanup;

assertErrorId(@() runFormationB4V46DevelopmentArmShard( ...
    1, 'v46-repaired-reference-a70-e05'), ...
    'FormationB4V46DevelopmentPermit:LauncherRequired');
launcherText = fileread(fullfile(pwd, ...
    'RUN', 'GA', 'launchFormationB4V46DevelopmentArmShard.sh'));
assert(~isempty(strfind(launcherText, '--no-init-all'))); %#ok<STREMP>
assert(~isempty(strfind(launcherText, '/usr/bin/env -i'))); %#ok<STREMP>
assert(~isempty(strfind(launcherText, ...
    'FORMATION_B4_V46_TRACKED_MATLAB_PATH'))); %#ok<STREMP>
assertMissingNoInitInvocationRejected(launcherText, resolution);

hookDirectory = fullfile(pwd, 'tmp', ...
    'codex_v46_ignored_package_hook_test');
hookPath = fullfile(hookDirectory, 'PKG_ADD');
mkdir(hookDirectory);
fid = fopen(hookPath, 'w');
assert(fid >= 0);
fprintf(fid, 'error(''ignored package hook executed'');\n');
fclose(fid);
hookCleanup = onCleanup(@() cleanupPackageHook( ...
    hookDirectory, hookPath));
assertErrorId(@() ...
    buildFormationB4V46DevelopmentExecutionSurfaceManifest(), ...
    'FormationB4V46DevelopmentPermit:AutomaticPathHookPresent');
clear hookCleanup;

previousGitDir = getenv('GIT_DIR');
gitDirCleanup = onCleanup(@() restoreEnvironmentVariable( ...
    'GIT_DIR', previousGitDir));
setenv('GIT_DIR', '/tmp/codex-v46-fake-gitdir');
assertErrorId(@() ...
    assertFormationB4V46DevelopmentCriticalFunctionPaths(), ...
    'FormationB4V46DevelopmentPermit:GitEnvironmentPresent');
clear gitDirCleanup;

[replaceListStatus, replaceListText] = builtin('system', sprintf( ...
    '/usr/bin/git --no-replace-objects -C %s replace --list', ...
    shellQuote(pwd)));
assert(replaceListStatus == 0);
assert(isempty(strtrim(replaceListText)));

previousReplaceBase = getenv('GIT_REPLACE_REF_BASE');
replaceCleanup = onCleanup(@() restoreEnvironmentVariable( ...
    'GIT_REPLACE_REF_BASE', previousReplaceBase));
setenv('GIT_REPLACE_REF_BASE', 'refs/codex-test-replace/');
assertErrorId(@() ...
    validateFormationB4V46DevelopmentExecutionGitState( ...
        repmat('0', 1, 40)), ...
    'FormationB4V46DevelopmentPermit:GitObjectReplacementPresent');
clear replaceCleanup;
fprintf('PASS: V46 development execution-surface manifest tests\n');
end

function assertMissingNoInitInvocationRejected(launcherText, resolution)
token = regexp(launcherText, '--eval "(.*)"', 'tokens', 'once');
assert(~isempty(token));
repoRoot = pwd;
directories = cell(size(resolution.relativePaths));
for pathIdx = 1:numel(resolution.relativePaths)
    directory = fileparts(resolution.relativePaths{pathIdx});
    if isempty(directory)
        directories{pathIdx} = repoRoot;
    else
        directories{pathIdx} = fullfile(repoRoot, ...
            strrep(directory, '/', filesep));
    end
end
trackedPath = strjoin(unique(sort(directories)), pathsep);
launcherPath = fullfile(repoRoot, 'RUN', 'GA', ...
    'launchFormationB4V46DevelopmentArmShard.sh');
command = sprintf([ ...
    '/usr/bin/env -i HOME=/Users/dex TMPDIR=/tmp ', ...
    'PATH=/opt/homebrew/bin:/usr/bin:/bin:/usr/sbin:/sbin ', ...
    'GIT_PAGER=cat ', ...
    'FORMATION_B4_V46_INVOCATION_CONTRACT=', ...
        'formation-b4-v46-development-invocation-v1 ', ...
    'FORMATION_B4_V46_LAUNCHER_PATH=%s ', ...
    'FORMATION_B4_V46_INIT_FILES_DISABLED=true ', ...
    'FORMATION_B4_V46_PREFLIGHT_GIT_IDENTITY_VERIFIED=true ', ...
    'FORMATION_B4_V46_PREFLIGHT_TRACKED_CLEAN=true ', ...
    ['FORMATION_B4_V46_PREFLIGHT_UNTRACKED_', ...
     'EXECUTABLES_ABSENT=true '], ...
    ['FORMATION_B4_V46_PREFLIGHT_GIT_POINTER_SHA256=', ...
     '3aa13c93b0e6f821363acd2337a72205843f9ea06be6c411b747c5afc9ef0ef0 '], ...
    'FORMATION_B4_V46_REPO_ROOT=%s ', ...
    'FORMATION_B4_V46_TRACKED_MATLAB_PATH=%s ', ...
    'FORMATION_B4_V46_CASE_ORDINAL=1 ', ...
    'FORMATION_B4_V46_ARM_ID=v46-repaired-reference-a70-e05 ', ...
    'FORMATION_B4_V46_OUTPUT_PATH='''' ', ...
    '/opt/homebrew/bin/octave --quiet --no-gui --eval %s 2>&1'], ...
    shellQuote(launcherPath), shellQuote(repoRoot), ...
    shellQuote(trackedPath), shellQuote(token{1}));
[status, output] = builtin('system', command);
assert(status ~= 0);
assert(~isempty(strfind(output, ...
    'exact sanitized launcher contract'))); %#ok<STREMP>
assert(isempty(strfind(output, ...
    'development arm execution remains sealed'))); %#ok<STREMP>
end

function cleanupPackageHook(directory, path)
if exist(path, 'file') ~= 0
    delete(path);
end
if exist(directory, 'dir') ~= 0
    rmdir(directory);
end
parent = fileparts(directory);
if exist(parent, 'dir') ~= 0
    entries = dir(parent);
    if numel(entries) == 2
        rmdir(parent);
    end
end
end

function restoreEnvironmentVariable(name, value)
if isempty(value)
    unsetenv(name);
else
    setenv(name, value);
end
end

function cleanupCheckerShadow(directory, path)
if exist(directory, 'dir') ~= 0
    try
        rmpath(directory);
    catch
    end
end
clear assertFormationB4V46DevelopmentCriticalFunctionPaths;
if exist(path, 'file') ~= 0
    delete(path);
end
if exist(directory, 'dir') ~= 0
    rmdir(directory);
end
end

function cleanupShadow(directory, path)
if exist(directory, 'dir') ~= 0
    try
        rmpath(directory);
    catch
    end
end
clear computeTextSha256;
if exist(path, 'file') ~= 0
    delete(path);
end
if exist(directory, 'dir') ~= 0
    rmdir(directory);
end
end

function assertErrorId(callback, expectedIdentifier)
actualIdentifier = '';
try
    callback();
catch errorInfo
    actualIdentifier = errorInfo.identifier;
end

assert(strcmp(actualIdentifier, expectedIdentifier), ...
    'Expected %s, received %s.', ...
    expectedIdentifier, actualIdentifier);
end

function value = shellQuote(value)
value = ['''', strrep(value, '''', '''"''"'''), ''''];
end
