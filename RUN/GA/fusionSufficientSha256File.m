function hashValue = fusionSufficientSha256File(path)
% FUSIONSUFFICIENTSHA256FILE Return lowercase SHA-256 for one file.

path = char(path);
if exist(path, 'file') ~= 2
    error('FusionSufficientHash:MissingFile', ...
        'Cannot hash missing file: %s', path);
end
command = sprintf('shasum -a 256 %s', shellQuote(path));
[status, output] = system(command);
if status ~= 0
    error('FusionSufficientHash:CommandFailed', ...
        'Unable to hash file: %s', path);
end
hashValue = regexp(lower(strtrim(output)), ...
    '^[0-9a-f]{64}', 'match', 'once');
if isempty(hashValue)
    error('FusionSufficientHash:InvalidOutput', ...
        'Invalid SHA-256 output for %s.', path);
end
end

function quoted = shellQuote(value)
value = char(value);
quoted = ['''', strrep(value, '''', '''"''"'''), ''''];
end
