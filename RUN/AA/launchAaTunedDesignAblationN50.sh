#!/bin/sh
set -eu

cd /Users/dex/Desktop/Code/Research/MULTISENSOR-LMB-FILTERS

python3 - <<'PY'
from datetime import datetime
import os
import subprocess

cwd = "/Users/dex/Desktop/Code/Research/MULTISENSOR-LMB-FILTERS"
stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
log_path = os.path.join(cwd, f"RUN/AA/AA_TUNED_DESIGN_ABLATION_N50_{stamp}.log")
pid_path = os.path.join(cwd, f"RUN/AA/AA_TUNED_DESIGN_ABLATION_N50_{stamp}.pid")

log = open(log_path, "w")
process = subprocess.Popen(
    ["octave", "--quiet", "RUN/AA/runAaTunedDesignAblationN50Validation.m"],
    cwd=cwd,
    stdin=subprocess.DEVNULL,
    stdout=log,
    stderr=subprocess.STDOUT,
    start_new_session=True,
)

with open(pid_path, "w") as pid_file:
    pid_file.write(f"{process.pid}\n")

print(f"Started AA tuned design N50 ablation with PID {process.pid}")
print(f"Log: {log_path}")
print(f"PID file: {pid_path}")
PY
