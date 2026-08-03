function state = resolveResearchGitState(repositoryRoot)
% RESOLVERESEARCHGITSTATE Record tracked dirt and untracked source files.

originalDirectory = pwd;
cleanup = []; %#ok<NASGU>
if nargin >= 1 && ~isempty(repositoryRoot)
    if ~ischar(repositoryRoot) || exist(repositoryRoot, 'dir') ~= 7
        error('ResearchGitState:InvalidRepositoryRoot', ...
            'repositoryRoot must name an existing directory.');
    end
    cd(repositoryRoot);
    cleanup = onCleanup(@() cd(originalDirectory));
end

[commitStatus, commit] = system('git rev-parse HEAD');
[rootStatus, resolvedRoot] = system('git rev-parse --show-toplevel');
[trackedStatus, trackedPorcelain] = system( ...
    'git status --porcelain --untracked-files=no');
[sourceStatus, untrackedSource] = system( ...
    'git ls-files --others --exclude-standard -- '':(glob)**/*.m''');
state = struct();
state.commit = strtrim(commit);
state.repositoryRoot = strtrim(resolvedRoot);
state.trackedWorktreeDirty = ...
    trackedStatus ~= 0 || ~isempty(strtrim(trackedPorcelain));
state.untrackedSourceFiles = splitLines(untrackedSource);
state.untrackedSourceDirty = ...
    sourceStatus ~= 0 || ~isempty(state.untrackedSourceFiles);
state.researchWorktreeDirty = ...
    state.trackedWorktreeDirty || state.untrackedSourceDirty;
if commitStatus ~= 0 || rootStatus ~= 0 || ...
        isempty(state.commit) || isempty(state.repositoryRoot)
    state.commit = 'unavailable';
    state.repositoryRoot = 'unavailable';
    state.researchWorktreeDirty = true;
end
end

function lines = splitLines(value)
value = strtrim(value);
if isempty(value)
    lines = {};
else
    lines = regexp(value, '\r?\n', 'split');
    lines = reshape(lines, 1, []);
end
end
