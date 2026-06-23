#!/usr/bin/env bash
set -euo pipefail

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

python3 - <<'PY'
from __future__ import annotations

from datetime import datetime
import glob
import os
import shlex
import subprocess
import sys


cwd = "/Users/dex/Desktop/Code/Research/MULTISENSOR-LMB-FILTERS"
allow_concurrent = os.environ.get("AA_ALLOW_CONCURRENT", "0") == "1"

active_pids: list[tuple[str, int]] = []
for pid_path in glob.glob(os.path.join(cwd, "RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_*.pid")):
    try:
        with open(pid_path, "r", encoding="utf-8") as fh:
            pid = int(fh.read().strip())
    except (OSError, ValueError):
        continue
    try:
        os.kill(pid, 0)
    except OSError:
        continue
    active_pids.append((pid_path, pid))

if active_pids and not allow_concurrent:
    print("Refusing to start held-out N50 while a TAES local-verifier N50 appears active:", file=sys.stderr)
    for pid_path, pid in active_pids:
        print(f"  PID {pid} from {pid_path}", file=sys.stderr)
    print("Set AA_ALLOW_CONCURRENT=1 only if concurrent Octave N50 runs are intentional.", file=sys.stderr)
    sys.exit(2)

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
log_path = os.path.join(cwd, f"RUN/AA/AA_TAES_HELDOUT_N50_BASESEED11_{stamp}.log")
pid_path = os.path.join(cwd, f"RUN/AA/AA_TAES_HELDOUT_N50_BASESEED11_{stamp}.pid")

octave_eval = (
    "addpath('RUN/AA'); "
    "aaControls=struct('saveMat', false, 'saveCheckpoints', false, "
    "'progressEverySteps', 0, 'existenceThreshold', 0.18); "
    "[reportPath, summary] = runAaBalancedCardinalityValidation(50, 11, true, "
    "aaControls, struct(), true, [9 18 19]); "
    "fprintf('AA_TAES_HELDOUT_N50_REPORT=%s\\n', reportPath); "
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

print(f"Started TAES held-out N50 baseSeed=11 run with PID {process.pid}")
print(f"Log: {log_path}")
print(f"PID file: {pid_path}")
print("Follow with:")
print(f"  tail -f {log_path}")
PY
