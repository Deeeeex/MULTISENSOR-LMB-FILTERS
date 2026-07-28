function digest = computeTextSha256(text)
% COMPUTETEXTSHA256 Cross-runtime SHA-256 for a character payload.

if isstring(text)
    if ~isscalar(text)
        error('SHA-256 text input must be scalar.');
    end
    text = char(text);
elseif ~ischar(text)
    error('SHA-256 text input must be character data.');
end
if exist('unicode2native', 'builtin') || ...
        exist('unicode2native', 'file')
    bytes = unicode2native(text, 'UTF-8');
else
    bytes = uint8(text);
end
digest = computeBytesSha256(bytes);
end
