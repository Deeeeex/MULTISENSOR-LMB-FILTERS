function factor = ...
    validateFormationReliableKlaContractionFactor(factor, role)
% VALIDATEFORMATIONRELIABLEKLACONTRACTIONFACTOR Fail closed on bad evidence.

if nargin < 2
    role = 'unspecified';
end
if ~ischar(role) || isempty(role) || ...
        ~isnumeric(factor) || ~isscalar(factor) || ...
        ~isreal(factor) || ~isfinite(factor) || factor < 0
    error('FormationKlaIndexEquivariance:InvalidContractionFactor', ...
        'The %s mean-square contraction factor is invalid.', ...
        safeRole(role));
end
end

function value = safeRole(role)
if nargin < 1 || ~ischar(role) || isempty(role)
    value = 'unspecified';
else
    value = role;
end
end
