function [groundTruth, measurements, groundTruthRfs, sensorTrajectories] = generateMultisensorGroundTruthEnhanced(model, varargin)
% GENERATEMULTISENSORGROUNDTRUTHENHANCED - Wrapper for extended motion support
%   [groundTruth, measurements, groundTruthRfs, sensorTrajectories] =
%   generateMultisensorGroundTruthEnhanced(model, varargin)
%
%   This wrapper delegates to generateMultisensorGroundTruth, which now
%   supports CV, CT, and Formation sensor motion as well as formation targets.
%
%   See also generateMultisensorGroundTruth

[groundTruth, measurements, groundTruthRfs, sensorTrajectories] = generateMultisensorGroundTruth(model, varargin{:});
end
