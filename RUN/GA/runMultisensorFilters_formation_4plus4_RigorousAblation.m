function [reportPath, summary] = runMultisensorFilters_formation_4plus4_RigorousAblation( ...
    numberOfTrials, baseSeed, useFixedSeed, writeReport, commConfigOverrides, ...
    adaptiveFusionOverrides, armSelection)
% RUNMULTISENSORFILTERS_FORMATION_4PLUS4_RIGOROUSABLATION
% Run the controlled reviewer-facing component ablation.
%
% Default arms 1:10 form the causal table:
%   Fixed -> Cov -> Link -> Exist -> Cov+Link -> shared three-factor
%   -> branch-decoupled -> structure-aware -> EMA/floor -> FID-FIA existence.
%
% Arm 11 is an optional deployment control matching the current
% Cardinality-critical mode. Include it explicitly with armSelection=1:11.

if nargin < 1 || isempty(numberOfTrials)
    numberOfTrials = 50;
end
if nargin < 2 || isempty(baseSeed)
    baseSeed = 1;
end
if nargin < 3 || isempty(useFixedSeed)
    useFixedSeed = true;
end
if nargin < 4 || isempty(writeReport)
    writeReport = true;
end
if nargin < 5 || isempty(commConfigOverrides)
    commConfigOverrides = struct();
end
if nargin < 6 || isempty(adaptiveFusionOverrides)
    adaptiveFusionOverrides = struct();
end
if nargin < 7 || isempty(armSelection)
    armSelection = 1:10;
end

scriptDir = fileparts(mfilename('fullpath'));
if ~isempty(scriptDir)
    addpath(scriptDir);
end

[reportPath, summary] = runMultisensorFilters_formation_4plus4_TieredLinkAblation( ...
    numberOfTrials, baseSeed, useFixedSeed, commConfigOverrides, writeReport, ...
    'rigorousComponentAblation', adaptiveFusionOverrides, armSelection);

summary.rigorousAblation.reviewerArmCount = 10;
summary.rigorousAblation.optionalDeploymentControlIndex = 11;
summary.rigorousAblation.selectedArmNames = summary.armNames;
end
