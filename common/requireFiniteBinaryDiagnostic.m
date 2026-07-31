function values = requireFiniteBinaryDiagnostic( ...
    values, expectedCount, diagnosticName)
% REQUIREFINITEBINARYDIAGNOSTIC Fail closed on missing safety diagnostics.

if nargin < 3 || isempty(diagnosticName)
    diagnosticName = 'diagnostic';
end
if ~(isnumeric(values) || islogical(values))
    error('RollingSafeCausalOption:InvalidDiagnostic', ...
        '%s must be numeric or logical.', diagnosticName);
end
values = reshape(values, 1, []);
numericValues = double(values);
if numel(values) ~= expectedCount || ...
        any(~isfinite(numericValues)) || ...
        any(numericValues ~= 0 & numericValues ~= 1)
    error('RollingSafeCausalOption:InvalidDiagnostic', ...
        '%s must contain exactly %d finite binary values.', ...
        diagnosticName, expectedCount);
end
values = logical(values);
end
