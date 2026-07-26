function count = countCanonicalGatewayControlPhases( ...
        modes, indices, requiredPhaseCount)
% COUNTCANONICALGATEWAYCONTROLPHASES Count preregistered phase labels.
%
% Only p1...pN count toward an N-phase control gate. This prevents
% behaviorally repeated labels such as p1/p7/p13 from satisfying the gate.

if nargin < 3 || ~isscalar(requiredPhaseCount) || ...
        ~isfinite(requiredPhaseCount) || requiredPhaseCount < 1 || ...
        mod(requiredPhaseCount, 1) ~= 0
    error('requiredPhaseCount must be a positive integer.');
end
if ischar(modes)
    modes = {modes};
end
modes = reshape(modes, 1, []);
indices = reshape(indices, 1, []);
if any(indices < 1 | indices > numel(modes) | ...
        mod(indices, 1) ~= 0)
    error('Gateway control indices are invalid.');
end

phases = [];
for cursor = indices
    token = regexp( ...
        char(modes{cursor}), '-p([0-9]+)-w[0-9]+$', ...
        'tokens', 'once');
    if ~isempty(token)
        phases(end + 1) = str2double(token{1}); %#ok<AGROW>
    end
end
canonicalPhases = 1:requiredPhaseCount;
count = numel(intersect(unique(phases), canonicalPhases));
end
