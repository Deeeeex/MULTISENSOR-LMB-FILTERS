% SETPATH -- Set up all the paths necessary to use the toolbox
%
% File guide:
%   This is the bootstrap script for interactive MATLAB sessions and demos.
%   Run it once before calling filters or scenario generators from the repo
%   root. It adds only source directories; generated reports and paper
%   assets are intentionally left off the MATLAB path.
rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, 'common'));
addpath(fullfile(rootDir, 'lmb'));
addpath(fullfile(rootDir, 'lmbm'));
addpath(fullfile(rootDir, 'marginalEvalulations'));
addpath(fullfile(rootDir, 'trials'));
addpath(fullfile(rootDir, 'multisensorLmb'));
addpath(fullfile(rootDir, 'multisensorLmbm'));
