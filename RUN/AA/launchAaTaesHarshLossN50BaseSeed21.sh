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
number_of_trials = int(os.environ.get("AA_STRESS_TRIALS", "50"))
base_seed = int(os.environ.get("AA_STRESS_BASE_SEED", "21"))

active_pids: list[tuple[str, int]] = []
for pattern in [
    "RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_*.pid",
    "RUN/AA/AA_TAES_HELDOUT_N50_BASESEED11_*.pid",
    "RUN/AA/AA_TAES_STRESS_HARSH_N*_BASESEED*_*.pid",
]:
    for pid_path in glob.glob(os.path.join(cwd, pattern)):
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
    print("Refusing to start harsh-loss N50 while another AA N50 run appears active:", file=sys.stderr)
    for pid_path, pid in active_pids:
        print(f"  PID {pid} from {pid_path}", file=sys.stderr)
    print("Set AA_ALLOW_CONCURRENT=1 only if concurrent Octave N50 runs are intentional.", file=sys.stderr)
    sys.exit(2)

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
log_path = os.path.join(
    cwd,
    f"RUN/AA/AA_TAES_STRESS_HARSH_N{number_of_trials}_BASESEED{base_seed}_{stamp}.log",
)
pid_path = os.path.join(
    cwd,
    f"RUN/AA/AA_TAES_STRESS_HARSH_N{number_of_trials}_BASESEED{base_seed}_{stamp}.pid",
)

octave_eval = (
    "addpath('RUN/AA'); "
    "aaControls=struct('saveMat', false, 'saveCheckpoints', false, "
    "'progressEverySteps', 0, 'existenceThreshold', 0.18); "
    "comm=struct('pDropLevels', [0.2 0.35 0.5 0.7], "
    "'pDropLevelCounts', [1 3 2 2]); "
    f"[reportPath, summary] = runAaBalancedCardinalityValidation({number_of_trials}, {base_seed}, true, "
    "aaControls, comm, true, [9 18 19]); "
    "fprintf('AA_TAES_STRESS_HARSH_REPORT=%s\\n', reportPath); "
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

print(f"Started harsh-loss AA stress run with PID {process.pid}")
print(f"Trials: {number_of_trials}")
print(f"Base seed: {base_seed}")
print("Packet-loss levels/counts: [0.2 0.35 0.5 0.7] / [1 3 2 2]")
print(f"Log: {log_path}")
print(f"PID file: {pid_path}")
print("Follow with:")
print(f"  tail -f {log_path}")
PY
