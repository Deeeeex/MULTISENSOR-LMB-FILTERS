function [hashValue, entries] = ...
    hashFusionSufficientRequiredSourcesAtCommit(projectRoot, commit)
% HASHFUSIONSUFFICIENTREQUIREDSOURCESATCOMMIT Hash required Git blobs.

projectRoot = char(projectRoot);
commit = char(commit);
if isempty(regexp(commit, '^[0-9a-f]{40}$', 'once'))
    error('FusionSufficientCommitHash:InvalidCommit', ...
        'Commit must be a lowercase 40-hex revision.');
end
paths = fusionSufficientRequiredSources();
entries = repmat(struct('path', '', 'sha256', ''), 1, numel(paths));
parts = cell(1, numel(paths));
for pathIdx = 1:numel(paths)
    temporaryPath = tempname();
    cleanup = onCleanup(@() deleteIfPresent(temporaryPath)); %#ok<NASGU>
    objectExpression = [commit, ':', paths{pathIdx}];
    command = sprintf('git -C %s show %s > %s', ...
        shellQuote(projectRoot), shellQuote(objectExpression), ...
        shellQuote(temporaryPath));
    [status, output] = system(command);
    if status ~= 0
        error('FusionSufficientCommitHash:MissingBlob', ...
            'Unable to read %s at commit %s: %s', ...
            paths{pathIdx}, commit, strtrim(output));
    end
    fileHash = fusionSufficientSha256File(temporaryPath);
    entries(pathIdx).path = paths{pathIdx};
    entries(pathIdx).sha256 = fileHash;
    parts{pathIdx} = sprintf('%s=%s\n', paths{pathIdx}, fileHash);
    clear cleanup;
end
hashValue = fusionSufficientSha256Text([parts{:}]);
end

function deleteIfPresent(path)
if exist(path, 'file') == 2
    delete(path);
end
end

function quoted = shellQuote(value)
value = char(value);
quoted = ['''', strrep(value, '''', '''"''"'''), ''''];
end
