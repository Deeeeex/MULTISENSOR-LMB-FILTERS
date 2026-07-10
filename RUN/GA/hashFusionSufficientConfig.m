function hashValue = hashFusionSufficientConfig(config)
% HASHFUSIONSUFFICIENTCONFIG SHA-256 of canonical frozen configuration.

if ~isstruct(config) || ~isscalar(config)
    error('FusionSufficientConfigHash:InvalidConfig', ...
        'Config must be a scalar struct.');
end
canonical = config;
excluded = {'configSha256', 'projectRoot', 'outputDirectory'};
for fieldIdx = 1:numel(excluded)
    if isfield(canonical, excluded{fieldIdx})
        canonical = rmfield(canonical, excluded{fieldIdx});
    end
end
hashValue = hashFusionSufficientValue(canonical);
end
