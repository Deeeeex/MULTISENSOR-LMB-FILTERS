#!/bin/sh
set -eu

cd /Users/dex/Desktop/Code/Research/MULTISENSOR-LMB-FILTERS

LOG_PATH="RUN/AA/AA_TUNED_HYBRID_VS_GA_N50_20260621_1516.log"

{
    echo "Launcher started at $(date '+%Y-%m-%d %H:%M:%S %Z')"
    echo "Working directory: $(pwd)"
    echo "Commit: $(git rev-parse HEAD)"
    if command -v caffeinate >/dev/null 2>&1; then
        caffeinate -dimsu octave --quiet RUN/AA/runAaHybridVsGaN50Validation.m
    else
        octave --quiet RUN/AA/runAaHybridVsGaN50Validation.m
    fi
    echo "Launcher finished at $(date '+%Y-%m-%d %H:%M:%S %Z')"
} > "$LOG_PATH" 2>&1
