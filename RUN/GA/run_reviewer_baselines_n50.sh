#!/bin/zsh
set -o pipefail

cd /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS-inffus || exit 1
mkdir -p RUN/GA/reviewer_baselines_validation
octave --quiet --eval "addpath('RUN/GA'); runReviewerBaselinesN50" 2>&1 \
  | tee -a RUN/GA/reviewer_baselines_validation/reviewer_baselines_n50.log
