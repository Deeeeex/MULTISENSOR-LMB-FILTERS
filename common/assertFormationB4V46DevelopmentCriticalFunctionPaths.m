function resolution = ...
    assertFormationB4V46DevelopmentCriticalFunctionPaths()
% ASSERTFORMATIONB4V46DEVELOPMENTCRITICALFUNCTIONPATHS Hermetic repo path.
%
% The filter calls a broad transitive MATLAB/Octave surface.  A short
% hand-maintained allowlist is therefore insufficient: an older checkout at
% the front of the path could replace an unlisted prediction, fusion, route,
% or hashing helper without changing this worktree.  This gate resolves
% every tracked non-test executable by its unique file basename and requires
% the selected implementation to be the exact file in this repository.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
gitIdentity = assertAnchoredGitIdentity(repoRoot);
[status, trackedText] = builtin('system', sprintf( ...
    '/usr/bin/git --no-replace-objects -C %s ls-files --cached', ...
    shellQuote(repoRoot)));
if status ~= 0
    error('FormationB4V46DevelopmentPermit:PathEnumerationFailed', ...
        'Tracked executable paths could not be enumerated.');
end
allPaths = splitNonemptyLines(trackedText);
relativePaths = cell(1, 0);
privateScopedPaths = cell(1, 0);
for pathIdx = 1:numel(allPaths)
    path = allPaths{pathIdx};
    executable = ~isempty(regexp(lower(path), ...
        '\.(m|p|mlx|oct|mex[^/]*)$', 'once'));
    testOnly = strncmp(path, 'tests/', numel('tests/'));
    privateScoped = ~isempty(strfind(path, '/private/')); %#ok<STREMP>
    if executable && ~testOnly && privateScoped
        privateScopedPaths{end + 1} = path; %#ok<AGROW>
    elseif executable && ~testOnly
        relativePaths{end + 1} = path; %#ok<AGROW>
    end
end
relativePaths = unique(sort(relativePaths));
privateScopedPaths = unique(sort(privateScopedPaths));
pathFunctionNames = cell(size(relativePaths));
for pathIdx = 1:numel(relativePaths)
    [~, pathFunctionNames{pathIdx}] = fileparts(relativePaths{pathIdx});
end
functionNames = unique(sort(pathFunctionNames));

resolvedPaths = cell(size(functionNames));
expectedPathGroups = cell(size(functionNames));
inactivePlatformVariant = false(size(functionNames));
pathResolvedWithoutParsing = false(size(functionNames));
for functionIdx = 1:numel(functionNames)
    matching = strcmp(pathFunctionNames, functionNames{functionIdx});
    expectedRelativePaths = relativePaths(matching);
    expectedPaths = cell(size(expectedRelativePaths));
    for pathIdx = 1:numel(expectedRelativePaths)
        expectedPaths{pathIdx} = fullfile(repoRoot, ...
            strrep(expectedRelativePaths{pathIdx}, '/', filesep));
    end
    expectedPathGroups{functionIdx} = expectedRelativePaths;
    extensions = cell(size(expectedRelativePaths));
    for pathIdx = 1:numel(expectedRelativePaths)
        [~, ~, extensions{pathIdx}] = fileparts( ...
            expectedRelativePaths{pathIdx});
    end
    mexVariantsOnly = all(cellfun(@(value) ...
        strncmp(lower(value), '.mex', numel('.mex')), extensions));
    try
        resolvedPaths{functionIdx} = resolveWithRuntimeBuiltin( ...
            functionNames{functionIdx});
    catch
        resolvedPaths{functionIdx} = resolveWithoutParsing( ...
            functionNames{functionIdx}, expectedPaths);
        pathResolvedWithoutParsing(functionIdx) = true;
    end
    if isempty(resolvedPaths{functionIdx}) && mexVariantsOnly
        inactivePlatformVariant(functionIdx) = true;
    elseif isempty(resolvedPaths{functionIdx}) || ...
            ~any(strcmp(resolvedPaths{functionIdx}, expectedPaths))
        error('FormationB4V46DevelopmentPermit:FunctionPathShadowed', ...
            ['Tracked executable %s resolved outside the frozen ', ...
             'repository or was absent from the MATLAB/Octave path.'], ...
            functionNames{functionIdx});
    end
end

resolution = struct();
resolution.contractVersion = ...
    'formation-b4-v46-development-hermetic-path-resolution-v2';
resolution.functionNames = functionNames;
resolution.relativePaths = relativePaths;
resolution.expectedPathGroups = expectedPathGroups;
resolution.resolvedPaths = resolvedPaths;
resolution.inactivePlatformVariantFunctionNames = ...
    functionNames(inactivePlatformVariant);
resolution.pathResolvedWithoutParsingFunctionNames = ...
    functionNames(pathResolvedWithoutParsing);
resolution.privateScopedPaths = privateScopedPaths;
resolution.allTrackedNonTestExecutablesCovered = true;
resolution.externalPathShadowingRejected = true;
resolution.privateExecutablesBoundByCallerDirectory = true;
resolution.gitIdentityContractVersion = gitIdentity.contractVersion;
resolution.gitWorktreePointerAnchored = true;
resolution.gitEnvironmentOverridesRejected = true;
resolution.canonicalSha256 = computeCanonicalValueSha256(struct( ...
    'contractVersion', resolution.contractVersion, ...
    'functionNames', {functionNames}, ...
    'relativePaths', {relativePaths}, ...
    'expectedPathGroups', {expectedPathGroups}, ...
    'resolvedPaths', {resolvedPaths}, ...
    'inactivePlatformVariantFunctionNames', ...
        {resolution.inactivePlatformVariantFunctionNames}, ...
    'pathResolvedWithoutParsingFunctionNames', ...
        {resolution.pathResolvedWithoutParsingFunctionNames}, ...
    'privateScopedPaths', {privateScopedPaths}, ...
    'gitIdentityContractVersion', ...
        resolution.gitIdentityContractVersion, ...
    'gitWorktreePointerAnchored', ...
        resolution.gitWorktreePointerAnchored, ...
    'gitEnvironmentOverridesRejected', ...
        resolution.gitEnvironmentOverridesRejected));
end

function resolved = resolveWithRuntimeBuiltin(functionName)
if exist('__which__', 'builtin') == 5
    metadata = builtin('__which__', functionName);
    resolved = metadata.file;
elseif exist('which', 'builtin') == 5
    resolved = builtin('which', functionName);
else
    handle = builtin('str2func', functionName);
    metadata = builtin('functions', handle);
    resolved = metadata.file;
end
end

function identity = assertAnchoredGitIdentity(repoRoot)
% The evidence run is intentionally bound to one physical linked worktree.
% This is the external trust root that prevents a fake Git directory from
% making altered source and a forged data-only history appear self-consistent.
expectedRepoRoot = ...
    ['/Users/dex/.config/superpowers/worktrees/', ...
     'MULTISENSOR-LMB-FILTERS/v46-development-permit'];
expectedGitDir = ...
    ['/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/.git/', ...
     'worktrees/v46-development-permit'];
expectedCommonDir = ...
    '/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/.git';
expectedPointer = sprintf('gitdir: %s\n', expectedGitDir);

if ~strcmp(canonicalPath(repoRoot), expectedRepoRoot)
    error('FormationB4V46DevelopmentPermit:GitIdentityDrift', ...
        'V46 execution is bound to its frozen physical worktree.');
end
gitPointerPath = fullfile(repoRoot, '.git');
[regularStatus, ~] = builtin('system', sprintf( ...
    '/bin/test -f %s && /bin/test ! -L %s', ...
    shellQuote(gitPointerPath), shellQuote(gitPointerPath)));
if regularStatus ~= 0 || ~strcmp(readExactFile(gitPointerPath), expectedPointer)
    error('FormationB4V46DevelopmentPermit:GitIdentityDrift', ...
        'The linked-worktree Git pointer changed or is not regular.');
end

[environmentStatus, environmentText] = ...
    builtin('system', '/usr/bin/env');
if environmentStatus ~= 0
    error('FormationB4V46DevelopmentPermit:GitIdentityFailed', ...
        'The process environment could not be enumerated.');
end
environmentLines = splitNonemptyLines(environmentText);
for lineIdx = 1:numel(environmentLines)
    separator = strfind(environmentLines{lineIdx}, '=');
    if isempty(separator)
        continue;
    end
    name = environmentLines{lineIdx}(1:separator(1) - 1);
    if strcmp(name, 'GIT_PAGER')
        continue;
    end
    if strcmp(name, 'GIT_REPLACE_REF_BASE')
        error( ...
            'FormationB4V46DevelopmentPermit:GitObjectReplacementPresent', ...
            'A custom Git replacement-ref namespace is not allowed.');
    end
    if strncmp(name, 'GIT_', numel('GIT_'))
        error('FormationB4V46DevelopmentPermit:GitEnvironmentPresent', ...
            'Git environment override %s is not allowed.', name);
    end
end

topLevel = runGitIdentityQuery(repoRoot, ...
    'rev-parse --path-format=absolute --show-toplevel');
gitDir = runGitIdentityQuery(repoRoot, ...
    'rev-parse --path-format=absolute --git-dir');
commonDir = runGitIdentityQuery(repoRoot, ...
    'rev-parse --path-format=absolute --git-common-dir');
indexPath = runGitIdentityQuery(repoRoot, ...
    'rev-parse --path-format=absolute --git-path index');
if ~strcmp(canonicalPath(topLevel), expectedRepoRoot) || ...
        ~strcmp(canonicalPath(gitDir), expectedGitDir) || ...
        ~strcmp(canonicalPath(commonDir), expectedCommonDir) || ...
        ~strcmp(canonicalPath(indexPath), ...
            canonicalPath(fullfile(expectedGitDir, 'index')))
    error('FormationB4V46DevelopmentPermit:GitIdentityDrift', ...
        'Git resolved a different worktree, gitdir, common dir, or index.');
end

identity = struct();
identity.contractVersion = ...
    'formation-b4-v46-development-git-identity-v1';
identity.repoRoot = expectedRepoRoot;
identity.gitDir = expectedGitDir;
identity.commonDir = expectedCommonDir;
identity.indexPath = fullfile(expectedGitDir, 'index');
identity.gitPointerRegular = true;
identity.gitPointerExact = true;
identity.gitEnvironmentOverridesRejected = true;
end

function value = runGitIdentityQuery(repoRoot, arguments)
[status, output] = builtin('system', sprintf( ...
    '/usr/bin/git --no-replace-objects -C %s %s', ...
    shellQuote(repoRoot), arguments));
if status ~= 0 || isempty(strtrim(output))
    error('FormationB4V46DevelopmentPermit:GitIdentityFailed', ...
        'A Git identity query failed.');
end
value = strtrim(output);
end

function value = readExactFile(path)
fid = fopen(path, 'rb');
if fid < 0
    error('FormationB4V46DevelopmentPermit:GitIdentityFailed', ...
        'The linked-worktree Git pointer could not be read.');
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
value = char(fread(fid, inf, '*uint8')');
end

function value = canonicalPath(value)
canonical = canonicalize_file_name(value);
if isempty(canonical)
    error('FormationB4V46DevelopmentPermit:GitIdentityFailed', ...
        'A required Git identity path could not be canonicalized.');
end
value = canonical;
end

function resolved = resolveWithoutParsing(functionName, expectedPaths)
directories = [{pwd}, regexp(path, pathsep, 'split')];
directories = unique(directories, 'stable');
resolved = '';
for directoryIdx = 1:numel(directories)
    directory = directories{directoryIdx};
    if isempty(directory)
        directory = pwd;
    elseif strcmp(directory, '.')
        directory = pwd;
    end
    [candidateNames, readError] = readdir(directory);
    if readError ~= 0
        continue;
    end
    for candidateIdx = 1:numel(candidateNames)
        candidateName = candidateNames{candidateIdx};
        if isempty(regexp(lower(candidateName), ...
                    ['^', regexptranslate('escape', lower(functionName)), ...
                     '\.(m|p|mlx|oct|mex[^/]*)$'], 'once'))
            continue;
        end
        candidatePath = fullfile(directory, candidateName);
        [candidateInfo, statError] = stat(candidatePath);
        if statError ~= 0 || S_ISDIR(candidateInfo.mode)
            continue;
        end
        if any(strcmp(candidatePath, expectedPaths))
            resolved = candidatePath;
            return;
        end
        error('FormationB4V46DevelopmentPermit:FunctionPathShadowed', ...
            'Tracked executable %s has an earlier external path file.', ...
            functionName);
    end
end
end

function lines = splitNonemptyLines(value)
value = strtrim(value);
if isempty(value)
    lines = cell(1, 0);
else
    lines = reshape(regexp(value, '\r?\n', 'split'), 1, []);
end
end

function value = shellQuote(value)
value = ['''', strrep(value, '''', '''"''"'''), ''''];
end
