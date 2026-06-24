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


def octave_string(value: object) -> str:
    text = str(value).replace("'", "''")
    return f"'{text}'"


cwd = "/Users/dex/Desktop/Code/Research/MULTISENSOR-LMB-FILTERS"
allow_concurrent = os.environ.get("AA_ALLOW_CONCURRENT", "0") == "1"
family = os.environ.get("AA_SCENARIO_FAMILY", "topology-ring").strip().lower()
number_of_trials = int(os.environ.get("AA_SCENARIO_TRIALS", "1"))
base_seed = int(os.environ.get("AA_SCENARIO_BASE_SEED", "31"))

families: dict[str, dict[str, object]] = {
    "topology-ring": {
        "label": "topology-ring-formation",
        "neighbor": "ring",
        "fov_half_angle": 60,
        "fov_range": 60000,
        "description": "same formation and packet-loss profile, sparse 8-node ring neighborhoods",
    },
    "partial-fov35": {
        "label": "partial-fov35-formation",
        "neighbor": "4plus4",
        "fov_half_angle": 35,
        "fov_range": 60000,
        "description": "same topology and packet-loss profile, narrower 35 degree sensor FOV",
    },
    "full-topology": {
        "label": "full-topology-formation",
        "neighbor": "full",
        "fov_half_angle": 60,
        "fov_range": 60000,
        "description": "same formation and packet-loss profile, complete communication neighborhoods",
    },
}

if family not in families:
    print(f"Unknown AA_SCENARIO_FAMILY={family!r}", file=sys.stderr)
    print(f"Available families: {', '.join(sorted(families))}", file=sys.stderr)
    sys.exit(2)

active_pids: list[tuple[str, int]] = []
for pattern in [
    "RUN/AA/AA_TAES_N50_LOCAL_VERIFIER_RERUN_*.pid",
    "RUN/AA/AA_TAES_HELDOUT_N50_BASESEED11_*.pid",
    "RUN/AA/AA_TAES_STRESS_HARSH_N*_BASESEED*_*.pid",
    "RUN/AA/AA_TAES_SCENARIO_*_N*_BASESEED*_*.pid",
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
    print("Refusing to start AA scenario-family run while another AA paper run appears active:", file=sys.stderr)
    for pid_path, pid in active_pids:
        print(f"  PID {pid} from {pid_path}", file=sys.stderr)
    print("Set AA_ALLOW_CONCURRENT=1 only if concurrent Octave runs are intentional.", file=sys.stderr)
    sys.exit(2)

cfg = families[family]
stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
safe_family = family.replace("-", "_")
log_path = os.path.join(
    cwd,
    f"RUN/AA/AA_TAES_SCENARIO_{safe_family}_N{number_of_trials}_BASESEED{base_seed}_{stamp}.log",
)
pid_path = os.path.join(
    cwd,
    f"RUN/AA/AA_TAES_SCENARIO_{safe_family}_N{number_of_trials}_BASESEED{base_seed}_{stamp}.pid",
)

octave_eval = (
    "addpath('RUN/AA'); "
    "aaControls=struct('saveMat', false, 'saveCheckpoints', false, "
    "'progressEverySteps', 0, 'existenceThreshold', 0.18); "
    "comm=struct(); "
    "scenario=struct("
    f"'scenarioLabel', {octave_string(cfg['label'])}, "
    f"'neighborMapMode', {octave_string(cfg['neighbor'])}, "
    f"'sensorFovHalfAngleDeg', {float(cfg['fov_half_angle'])}, "
    f"'sensorFovRange', {float(cfg['fov_range'])}); "
    f"[reportPath, summary] = runAaBalancedCardinalityValidation({number_of_trials}, {base_seed}, true, "
    "aaControls, comm, true, [9 18 19], struct(), scenario); "
    "fprintf('AA_TAES_SCENARIO_REPORT=%s\\n', reportPath); "
    "disp(summary.scenarioControls); "
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

print(f"Started AA scenario-family run with PID {process.pid}")
print(f"Scenario family: {family}")
print(f"Description: {cfg['description']}")
print(f"Trials: {number_of_trials}")
print(f"Base seed: {base_seed}")
print("Arms: [9 18 19]")
print(f"Log: {log_path}")
print(f"PID file: {pid_path}")
print("Follow with:")
print(f"  tail -f {log_path}")
PY
