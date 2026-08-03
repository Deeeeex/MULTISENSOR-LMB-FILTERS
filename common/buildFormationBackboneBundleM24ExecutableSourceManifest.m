function manifest = ...
    buildFormationBackboneBundleM24ExecutableSourceManifest()
% BUILDFORMATIONBACKBONEBUNDLEM24EXECUTABLESOURCEMANIFEST Hash run surface.
%
% The protocol registry itself and tests are excluded so a later data-only
% freeze can register this manifest without a self-referential digest.

repoRoot = fileparts(fileparts(mfilename('fullpath')));
[status, trackedText] = system(sprintf( ...
    'git -C %s ls-files --cached', shellQuote(repoRoot)));
if status ~= 0
    error('FormationBundleSourcePermit:SourceManifestFailed', ...
        'Tracked executable source could not be enumerated.');
end
[status, repositoryText] = system(sprintf([ ...
    'find %s \\( -type f -o -type l \\) ', ...
    '\\( -name ''*.m'' -o -name ''*.p'' ', ...
    '-o -name ''*.mlx'' -o -name ''*.oct'' ', ...
    '-o -name ''*.mex*'' \\) ', ...
    '-not -path %s -print'], ...
    shellQuote(repoRoot), ...
    shellQuote(fullfile(repoRoot, '.git', '*'))));
if status ~= 0
    error('FormationBundleSourcePermit:SourceManifestFailed', ...
        'Repository executable source could not be enumerated.');
end
trackedAllPaths = splitNonemptyLines(trackedText);
repositoryPaths = splitNonemptyLines(repositoryText);
rootPrefix = [repoRoot, filesep];
for pathIdx = 1:numel(repositoryPaths)
    if strncmp(repositoryPaths{pathIdx}, ...
            rootPrefix, numel(rootPrefix))
        repositoryPaths{pathIdx} = ...
            repositoryPaths{pathIdx}(numel(rootPrefix) + 1:end);
    end
end
trackedExecutablePaths = filterExecutablePaths( ...
    trackedAllPaths, false);
repositoryExecutablePaths = filterExecutablePaths( ...
    repositoryPaths, false);
untrackedPaths = setdiff( ...
    unique(sort(repositoryExecutablePaths)), ...
    unique(sort(trackedExecutablePaths)));
if ~isempty(untrackedPaths)
    error('FormationBundleSourcePermit:UnregisteredExecutableSource', ...
        'Untracked executable source is present: %s.', ...
        untrackedPaths{1});
end
paths = filterExecutablePaths( ...
    trackedAllPaths, true);
paths = unique(sort(paths));
sha256 = cell(size(paths));
for pathIdx = 1:numel(paths)
    absolutePath = fullfile(repoRoot, paths{pathIdx});
    if exist(absolutePath, 'file') == 0
        error('FormationBundleSourcePermit:SourceManifestFailed', ...
            'A tracked executable source file disappeared.');
    end
    sha256{pathIdx} = computeFileSha256(absolutePath);
end
payload = struct();
payload.contractVersion = ...
    'formation-backbone-bundle-m24-executable-source-manifest-v1';
payload.exclusionContractVersion = ...
    'registry-and-tests-excluded-v1';
payload.paths = reshape(paths, 1, []);
payload.sha256 = reshape(sha256, 1, []);
payload.untrackedExecutablePaths = cell(1, 0);
manifest = payload;
manifest.canonicalSha256 = ...
    computeCanonicalValueSha256(payload);
end

function paths = filterExecutablePaths(paths, excludeRegistryAndTests)
keep = false(size(paths));
for pathIdx = 1:numel(paths)
    path = paths{pathIdx};
    executable = ~isempty(regexp(lower(path), ...
        '\.(m|p|mlx|oct|mex[^/]*)$', 'once'));
    excluded = excludeRegistryAndTests && ( ...
        strncmp(path, 'tests/', numel('tests/')) || ...
        strcmp(path, [ ...
            'common/', ...
            'getFormationBackboneBundleM24DevelopmentProtocol.m']));
    keep(pathIdx) = executable && ~excluded;
end
paths = paths(keep);
end

function lines = splitNonemptyLines(value)
value = strtrim(value);
if isempty(value)
    lines = cell(1, 0);
else
    lines = regexp(value, '\r?\n', 'split');
    lines = reshape(lines, 1, []);
end
end

function value = shellQuote(value)
value = ['''', strrep(value, '''', '''"''"'''), ''''];
end
