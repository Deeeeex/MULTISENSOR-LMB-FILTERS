function [hashValue, entries] = ...
    hashFusionSufficientRequiredSources(projectRoot)
% HASHFUSIONSUFFICIENTREQUIREDSOURCES Hash the frozen source manifest.

projectRoot = char(projectRoot);
paths = fusionSufficientRequiredSources();
entries = repmat(struct('path', '', 'sha256', ''), 1, numel(paths));
parts = cell(1, numel(paths));
for pathIdx = 1:numel(paths)
    absolutePath = fullfile(projectRoot, paths{pathIdx});
    if exist(absolutePath, 'file') ~= 2
        error('FusionSufficientSourceHash:MissingSource', ...
            'Required source is missing: %s', paths{pathIdx});
    end
    fileHash = sha256File(absolutePath);
    entries(pathIdx).path = paths{pathIdx};
    entries(pathIdx).sha256 = fileHash;
    parts{pathIdx} = sprintf('%s=%s\n', paths{pathIdx}, fileHash);
end
hashValue = sha256Text([parts{:}]);
end

function hashValue = sha256File(path)
command = sprintf('shasum -a 256 %s', shellQuote(path));
[status, output] = system(command);
if status ~= 0
    error('FusionSufficientSourceHash:HashFailed', ...
        'Unable to hash required source: %s', path);
end
hashValue = parseHash(output, path);
end

function hashValue = sha256Text(value)
path = tempname();
cleanup = onCleanup(@() deleteIfPresent(path)); %#ok<NASGU>
fid = fopen(path, 'wb');
if fid < 0
    error('FusionSufficientSourceHash:TemporaryFile', ...
        'Unable to open temporary manifest input.');
end
fileCleanup = onCleanup(@() fclose(fid));
bytes = uint8(value);
count = fwrite(fid, bytes, 'uint8');
if count ~= numel(bytes)
    error('FusionSufficientSourceHash:TemporaryFile', ...
        'Unable to write complete source manifest.');
end
clear fileCleanup;
command = sprintf('shasum -a 256 %s', shellQuote(path));
[status, output] = system(command);
if status ~= 0
    error('FusionSufficientSourceHash:HashFailed', ...
        'Unable to hash canonical source manifest.');
end
hashValue = parseHash(output, 'source manifest');
end

function hashValue = parseHash(output, description)
hashValue = regexp(lower(strtrim(output)), ...
    '^[0-9a-f]{64}', 'match', 'once');
if isempty(hashValue)
    error('FusionSufficientSourceHash:InvalidHash', ...
        'Invalid SHA-256 output for %s.', description);
end
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
