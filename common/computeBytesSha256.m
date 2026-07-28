function digest = computeBytesSha256(bytes)
% COMPUTEBYTESSHA256 Cross-runtime SHA-256 for an exact byte sequence.

if ~isnumeric(bytes) && ~islogical(bytes)
    error('SHA-256 byte input must be numeric.');
end
bytes = reshape(uint8(bytes), 1, []);
if exist('OCTAVE_VERSION', 'builtin')
    digest = lower(hash('sha256', char(bytes)));
    return;
end

try
    engine = javaMethod( ...
        'getInstance', ...
        'java.security.MessageDigest', 'SHA-256');
    engine.update(bytes);
    raw = engine.digest();
    unsigned = uint8(mod(double(raw), 256));
    hex = lower(dec2hex(unsigned, 2));
    digest = reshape(hex', 1, []);
catch errorInfo
    error('SHA-256 is unavailable in this runtime: %s', ...
        errorInfo.message);
end
end
