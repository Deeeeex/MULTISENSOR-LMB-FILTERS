#!/bin/zsh
set -o pipefail

cd /Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS-inffus || exit 1
octave --quiet --eval "addpath('RUN/GA'); runGospaCoreN50" 2>&1 \
  | tee -a RUN/GA/gospa_validation/gospa_core_n50.log
