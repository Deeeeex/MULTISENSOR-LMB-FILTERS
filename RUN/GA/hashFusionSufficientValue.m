function hashValue = hashFusionSufficientValue(value)
% HASHFUSIONSUFFICIENTVALUE SHA-256 of recursively ordered JSON data.

try
    canonical = canonicalizeValue(value);
    encoded = jsonencode(canonical);
catch exception
    error('FusionSufficientValueHash:EncodeFailed', ...
        'Unable to encode canonical value: %s', exception.message);
end
hashValue = sha256Bytes(uint8(encoded));
end

function value = canonicalizeValue(value)
if isstruct(value)
    value = orderfields(value);
    fields = fieldnames(value);
    for elementIdx = 1:numel(value)
        for fieldIdx = 1:numel(fields)
            name = fields{fieldIdx};
            value(elementIdx).(name) = ...
                canonicalizeValue(value(elementIdx).(name));
        end
    end
elseif iscell(value)
    for elementIdx = 1:numel(value)
        value{elementIdx} = canonicalizeValue(value{elementIdx});
    end
end
end

function hashValue = sha256Bytes(bytes)
path = tempname();
cleanup = onCleanup(@() deleteIfPresent(path)); %#ok<NASGU>
fid = fopen(path, 'wb');
if fid < 0
    error('FusionSufficientValueHash:TemporaryFile', ...
        'Unable to open temporary hash input.');
end
fileCleanup = onCleanup(@() fclose(fid));
count = fwrite(fid, bytes, 'uint8');
if count ~= numel(bytes)
    error('FusionSufficientValueHash:TemporaryFile', ...
        'Unable to write complete temporary hash input.');
end
clear fileCleanup;
command = sprintf('shasum -a 256 %s', shellQuote(path));
[status, output] = system(command);
if status ~= 0
    error('FusionSufficientValueHash:HashFailed', ...
        'Unable to hash canonical value.');
end
hashValue = regexp(lower(strtrim(output)), ...
    '^[0-9a-f]{64}', 'match', 'once');
if isempty(hashValue)
    error('FusionSufficientValueHash:InvalidHash', ...
        'Invalid SHA-256 output for canonical value.');
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
