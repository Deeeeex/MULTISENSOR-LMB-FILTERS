function linkFusionSufficientFileNoClobber(sourcePath, finalPath)
% LINKFUSIONSUFFICIENTFILENOCLOBBER Atomically publish via POSIX link(2).

sourcePath = char(sourcePath);
finalPath = char(finalPath);
if exist(sourcePath, 'file') ~= 2
    error('FusionSufficientNoClobber:MissingSource', ...
        'Publication source is missing: %s', sourcePath);
end
if exist(finalPath, 'file') == 2 || exist(finalPath, 'dir') == 7
    error('FusionSufficientNoClobber:DestinationExists', ...
        'Refusing to overwrite publication target: %s', finalPath);
end
command = sprintf('/bin/ln %s %s', ...
    shellQuote(sourcePath), shellQuote(finalPath));
[status, output] = system(command);
if status ~= 0
    error('FusionSufficientNoClobber:LinkFailed', ...
        'Atomic no-clobber publication failed for %s: %s', ...
        finalPath, strtrim(output));
end
end

function quoted = shellQuote(value)
value = char(value);
quoted = ['''', strrep(value, '''', '''"''"'''), ''''];
end
