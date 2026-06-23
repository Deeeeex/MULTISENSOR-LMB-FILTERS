#!/usr/bin/env bash
set -euo pipefail

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

python3 - <<'PY'
from __future__ import annotations

from datetime import datetime
import os
import shlex
import subprocess


cwd = "/Users/dex/Desktop/Code/Research/MULTISENSOR-LMB-FILTERS"
stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
log_path = os.path.join(cwd, f"RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_{stamp}.log")
pid_path = os.path.join(cwd, f"RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_{stamp}.pid")

octave_eval = (
    "addpath('RUN/AA'); "
    "aaControls=struct('saveMat', false, 'saveCheckpoints', false, "
    "'progressEverySteps', 0, 'existenceThreshold', 0.18); "
    "[reportPath, summary] = runAaBalancedCardinalityValidation(50, 1, true, "
    "aaControls, struct(), true, [9 18 19]); "
    "fprintf('AA_TAES_N50_LOCAL_VERIFIER_REPORT=%s\\n', reportPath); "
    "disp(summary.consensus); "
    "disp(summary.local.meanAcrossSensors);"
)

command = (
    "set -o pipefail; "
    f"octave --quiet --eval {shlex.quote(octave_eval)} 2>&1 | tee {shlex.quote(log_path)}"
)
process = subprocess.Popen(
    ["bash", "-lc", command],
    cwd=cwd,
    stdin=subprocess.DEVNULL,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
    start_new_session=True,
)

with open(pid_path, "w", encoding="utf-8") as fh:
    fh.write(f"{process.pid}\n")

print(f"Started TAES N50 local-verifier rerun with PID {process.pid}")
print(f"Log: {log_path}")
print(f"PID file: {pid_path}")
print("Follow with:")
print(f"  tail -f {log_path}")
PY
