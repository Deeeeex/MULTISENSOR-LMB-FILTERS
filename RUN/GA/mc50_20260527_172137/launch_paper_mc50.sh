#!/usr/bin/env bash
set -u

PROJECT_ROOT="/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS"
RUN_DIR="$PROJECT_ROOT/RUN/GA/mc50_20260527_172137"
LOG="$RUN_DIR/paper_mc50.log"
PIDFILE="$RUN_DIR/paper_mc50.pid"

cd "$PROJECT_ROOT"
echo "$$" > "$PIDFILE"
exec /opt/homebrew/bin/octave-cli --quiet --path "$RUN_DIR" --eval "run_paper_mc50_batch" >> "$LOG" 2>&1
