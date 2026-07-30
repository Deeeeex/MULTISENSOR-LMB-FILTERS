function [sourcePath, dataset] = ...
    generateLabelSetCcwReturnSourceShardM24(seed, options)
% GENERATELABELSETCCWRETURNSOURCESHARDM24 Legacy frozen-CCW wrapper.

if nargin < 2 || isempty(options)
    options = struct();
end
options.legacyCcwContract = true;
[sourcePath, dataset] = ...
    generateLabelSetBehaviorReturnSourceShardM24( ...
        seed, 'fixed-counter-clockwise', options);
end
