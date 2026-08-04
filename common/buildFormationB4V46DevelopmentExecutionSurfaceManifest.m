function manifest = ...
    buildFormationB4V46DevelopmentExecutionSurfaceManifest()
% BUILDFORMATIONB4V46DEVELOPMENTEXECUTIONSURFACEMANIFEST Hash executable code.
%
% Every tracked executable source file outside tests is included.  Permit
% activation is stored as non-executable JSON, so a later data-only commit
% can bind this digest without creating a self-referential code hash.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
assertFormationB4V46DevelopmentCriticalFunctionPaths();
[status, trackedText] = builtin('system', sprintf( ...
    '/usr/bin/git --no-replace-objects -C %s ls-files --cached', ...
    shellQuote(repoRoot)));
if status ~= 0
    error('FormationB4V46DevelopmentPermit:ManifestFailed', ...
        'Tracked executable source could not be enumerated.');
end
[status, repositoryText] = builtin('system', sprintf([ ...
    '/usr/bin/find %s \\( -type f -o -type l \\) ', ...
    '\\( -name ''*.m'' -o -name ''*.p'' ', ...
    '-o -name ''*.mlx'' -o -name ''*.oct'' ', ...
    '-o -name ''*.mex*'' ', ...
    '-o -name ''launchFormationB4V46DevelopmentArmShard.sh'' ', ...
    '-o -name ''PKG_ADD'' -o -name ''PKG_DEL'' ', ...
    '\\) -print'], ...
    shellQuote(repoRoot)));
if status ~= 0
    error('FormationB4V46DevelopmentPermit:ManifestFailed', ...
        'Repository executable source could not be enumerated.');
end
trackedAll = splitNonemptyLines(trackedText);
repositoryAll = splitNonemptyLines(repositoryText);
prefix = [repoRoot, filesep];
for pathIdx = 1:numel(repositoryAll)
    if strncmp(repositoryAll{pathIdx}, prefix, numel(prefix))
        repositoryAll{pathIdx} = ...
            repositoryAll{pathIdx}(numel(prefix) + 1:end);
    end
end
trackedExecutable = filterExecutablePaths(trackedAll, false);
repositoryExecutable = filterExecutablePaths(repositoryAll, false);
packageHooks = repositoryAll(cellfun(@(value) ...
    ~isempty(regexp(value, '(^|/)(PKG_ADD|PKG_DEL)$', 'once')), ...
    repositoryAll));
if ~isempty(packageHooks)
    error('FormationB4V46DevelopmentPermit:AutomaticPathHookPresent', ...
        'Automatic MATLAB/Octave path hooks are not allowed: %s.', ...
        packageHooks{1});
end
untracked = setdiff(unique(sort(repositoryExecutable)), ...
    unique(sort(trackedExecutable)));
if ~isempty(untracked)
    error('FormationB4V46DevelopmentPermit:UntrackedExecutable', ...
        'Untracked executable source is present: %s.', untracked{1});
end
paths = unique(sort(filterExecutablePaths(trackedAll, true)));
sha256 = cell(size(paths));
for pathIdx = 1:numel(paths)
    absolutePath = fullfile(repoRoot, paths{pathIdx});
    if exist(absolutePath, 'file') == 0
        error('FormationB4V46DevelopmentPermit:ManifestFailed', ...
            'A tracked executable source file disappeared.');
    end
    sha256{pathIdx} = computeFileSha256(absolutePath);
end
payload = struct();
payload.contractVersion = ...
    'formation-b4-v46-development-execution-surface-manifest-v1';
payload.exclusionContractVersion = 'tests-only-excluded-v1';
payload.paths = reshape(paths, 1, []);
payload.sha256 = reshape(sha256, 1, []);
payload.untrackedExecutablePaths = cell(1, 0);
manifest = payload;
manifest.canonicalSha256 = computeCanonicalValueSha256(payload);
end

function paths = filterExecutablePaths(paths, excludeTests)
keep = false(size(paths));
for pathIdx = 1:numel(paths)
    path = paths{pathIdx};
    executable = ~isempty(regexp(lower(path), ...
        '\.(m|p|mlx|oct|mex[^/]*)$', 'once')) || ...
        strcmp(path, ...
            'RUN/GA/launchFormationB4V46DevelopmentArmShard.sh') || ...
        ~isempty(regexp(path, '(^|/)(PKG_ADD|PKG_DEL)$', 'once'));
    excluded = excludeTests && ...
        strncmp(path, 'tests/', numel('tests/'));
    keep(pathIdx) = executable && ~excluded;
end
paths = paths(keep);
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
