function test_formation_b4_v46_development_no_replace_publication()
% Atomic publication succeeds once and never replaces an existing shard.

directory = tempname();
mkdir(directory);
cleanup = onCleanup(@() cleanupDirectory(directory));
temporaryPath = fullfile(directory, 'temporary.mat');
outputPath = fullfile(directory, 'shard.mat');
writeBytes(temporaryPath, uint8('first-shard'));
publishFormationB4V46DevelopmentArmShardNoReplace( ...
    temporaryPath, outputPath);
assert(exist(temporaryPath, 'file') == 0);
assert(strcmp(readBytes(outputPath), 'first-shard'));

secondTemporaryPath = fullfile(directory, 'temporary-2.mat');
writeBytes(secondTemporaryPath, uint8('second-shard'));
assertErrorId(@() ...
    publishFormationB4V46DevelopmentArmShardNoReplace( ...
        secondTemporaryPath, outputPath), ...
    'FormationB4V46DevelopmentShard:OutputExists');
assert(exist(secondTemporaryPath, 'file') ~= 0);
assert(strcmp(readBytes(outputPath), 'first-shard'));
clear cleanup;
fprintf('PASS: V46 atomic no-replace publication tests\n');
end

function writeBytes(path, bytes)
fid = fopen(path, 'wb');
assert(fid >= 0);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
count = fwrite(fid, bytes, 'uint8');
assert(count == numel(bytes));
end

function value = readBytes(path)
fid = fopen(path, 'rb');
assert(fid >= 0);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
value = char(fread(fid, inf, '*uint8')');
end

function cleanupDirectory(directory)
if exist(directory, 'dir') ~= 0
    files = dir(directory);
    for idx = 1:numel(files)
        if ~files(idx).isdir
            delete(fullfile(directory, files(idx).name));
        end
    end
    rmdir(directory);
end
end

function assertErrorId(callback, expectedIdentifier)
actualIdentifier = '';
try
    callback();
catch errorInfo
    actualIdentifier = errorInfo.identifier;
end
assert(strcmp(actualIdentifier, expectedIdentifier), ...
    'Expected %s, received %s.', ...
    expectedIdentifier, actualIdentifier);
end
