#!/bin/zsh
set -o pipefail

cd /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS-inffus || exit 1
mkdir -p RUN/GA/gospa_validation
octave --quiet --eval "addpath('RUN/GA'); runPdWeightedGospaN50" 2>&1 \
  | tee -a RUN/GA/gospa_validation/gospa_pd_weighted_n50.log
