function digest = computeFileSha256(path)
% COMPUTEFILESHA256 Hash a file without changing it.

fid = fopen(path, 'rb');
if fid < 0
    error('Could not hash file: %s', path);
end
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
bytes = fread(fid, inf, '*uint8');
digest = computeBytesSha256(bytes);
end
