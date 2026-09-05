function [spatialPresent, existencePresent, informed, excludedCount] = ...
        restrictUntouchedLmbPriorsV284( ...
            objects, spatialPresent, existencePresent, existenceWeights, ...
            existenceOverrides)
% Restrict only when at least one actual input has an observation lineage.
% Observable absent-label censors remain legitimate negative information.
flags = false(size(existencePresent));
for j = 1:numel(objects)
    if isempty(objects{j})
        flags(j) = existencePresent(j) && isfinite(existenceOverrides(j));
    else
        assert(isfield(objects{j}, 'hasObservationLineage'), ...
            'V284 requires transmitted lineage metadata, not inferred flags.');
        flags(j) = logical(objects{j}.hasObservationLineage);
    end
end
active = existencePresent & existenceWeights > 1e-12;
informed = any(active & flags);
excludedCount = 0;
if informed
    excludedCount = nnz(active & ~flags);
    spatialPresent = spatialPresent & flags;
    existencePresent = existencePresent & flags;
end
end
