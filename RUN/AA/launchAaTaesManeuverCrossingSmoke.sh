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
number_of_trials = int(os.environ.get("AA_CROSSING_TRIALS", "5"))
base_seed = int(os.environ.get("AA_CROSSING_BASE_SEED", "71"))
crossing_window = os.environ.get("AA_CROSSING_WINDOW", "[9 17]").strip()

active_pids: list[tuple[str, int]] = []
for pattern in [
    "RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_*.pid",
    "RUN/AA/AA_TAES_HELDOUT_N50_BASESEED*_*.pid",
    "RUN/AA/AA_TAES_STRESS_HARSH_N*_BASESEED*_*.pid",
    "RUN/AA/AA_TAES_SCENARIO_*_N*_BASESEED*_*.pid",
    "RUN/AA/AA_TAES_CROSSING_N*_BASESEED*_*.pid",
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
    print("Refusing to start maneuver-crossing AA run while another AA paper run appears active:", file=sys.stderr)
    for pid_path, pid in active_pids:
        print(f"  PID {pid} from {pid_path}", file=sys.stderr)
    print("Set AA_ALLOW_CONCURRENT=1 only if concurrent Octave runs are intentional.", file=sys.stderr)
    sys.exit(2)

stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
log_path = os.path.join(
    cwd,
    f"RUN/AA/AA_TAES_CROSSING_N{number_of_trials}_BASESEED{base_seed}_{stamp}.log",
)
pid_path = os.path.join(
    cwd,
    f"RUN/AA/AA_TAES_CROSSING_N{number_of_trials}_BASESEED{base_seed}_{stamp}.pid",
)

octave_eval = (
    "addpath('RUN/AA'); "
    "aaControls=struct('saveMat', false, 'saveCheckpoints', false, "
    "'progressEverySteps', 0, 'existenceThreshold', 0.18); "
    "scenario=struct('scenarioLabel', 'maneuver-crossing-assignment', "
    "'targetScenarioMode', 'maneuver-crossing-assignment', "
    f"'crossingWindow', {crossing_window}); "
    f"[reportPath, summary] = runAaBalancedCardinalityValidation({number_of_trials}, {base_seed}, true, "
    "aaControls, struct(), true, [9 18 19], struct(), scenario); "
    "fprintf('AA_TAES_CROSSING_REPORT=%s\\n', reportPath); "
    "disp(summary.scenarioControls); "
    "disp(summary.scenarioWindow.consensus); "
    "disp(summary.scenarioWindow.local.meanAcrossSensors);"
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

print(f"Started TAES maneuver-crossing AA run with PID {process.pid}")
print("Scenario: maneuver-crossing-assignment")
print(f"Trials: {number_of_trials}")
print(f"Base seed: {base_seed}")
print(f"Crossing window: {crossing_window}")
print("Arms: [9 18 19]")
print(f"Log: {log_path}")
print(f"PID file: {pid_path}")
print("Follow with:")
print(f"  tail -f {log_path}")
PY
