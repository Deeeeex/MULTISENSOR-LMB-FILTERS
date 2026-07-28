function state = resolveResearchGitState()
% RESOLVERESEARCHGITSTATE Record tracked dirt and untracked source files.

[commitStatus, commit] = system('git rev-parse HEAD');
[trackedStatus, trackedPorcelain] = system( ...
    'git status --porcelain --untracked-files=no');
[sourceStatus, untrackedSource] = system( ...
    'git ls-files --others --exclude-standard -- '':(glob)**/*.m''');
state = struct();
state.commit = strtrim(commit);
state.trackedWorktreeDirty = ...
    trackedStatus ~= 0 || ~isempty(strtrim(trackedPorcelain));
state.untrackedSourceFiles = splitLines(untrackedSource);
state.untrackedSourceDirty = ...
    sourceStatus ~= 0 || ~isempty(state.untrackedSourceFiles);
state.researchWorktreeDirty = ...
    state.trackedWorktreeDirty || state.untrackedSourceDirty;
if commitStatus ~= 0 || isempty(state.commit)
    state.commit = 'unavailable';
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
