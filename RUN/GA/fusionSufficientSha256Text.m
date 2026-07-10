function hashValue = fusionSufficientSha256Text(value)
% FUSIONSUFFICIENTSHA256TEXT SHA-256 of exact UTF-8-compatible bytes.

path = tempname();
cleanup = onCleanup(@() deleteIfPresent(path)); %#ok<NASGU>
fid = fopen(path, 'wb');
if fid < 0
    error('FusionSufficientTextHash:TemporaryFile', ...
        'Unable to open temporary text hash input.');
end
fileCleanup = onCleanup(@() fclose(fid));
bytes = uint8(char(value));
count = fwrite(fid, bytes, 'uint8');
if count ~= numel(bytes)
    error('FusionSufficientTextHash:TemporaryFile', ...
        'Unable to write complete text hash input.');
end
clear fileCleanup;
hashValue = fusionSufficientSha256File(path);
end

function deleteIfPresent(path)
if exist(path, 'file') == 2
    delete(path);
end
end
