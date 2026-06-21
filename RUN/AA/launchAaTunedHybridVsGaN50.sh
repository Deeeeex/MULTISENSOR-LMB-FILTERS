#!/bin/sh
set -eu

cd /Users/dex/Desktop/Code/Research/MULTISENSOR-LMB-FILTERS

python3 - <<'PY'
import os
import subprocess

cwd = "/Users/dex/Desktop/Code/Research/MULTISENSOR-LMB-FILTERS"
log_path = os.path.join(cwd, "RUN/AA/AA_TUNED_HYBRID_VS_GA_N50_20260621_1516.log")
pid_path = os.path.join(cwd, "RUN/AA/AA_TUNED_HYBRID_VS_GA_N50_20260621_1516.pid")

log = open(log_path, "w")
process = subprocess.Popen(
    ["octave", "--quiet", "RUN/AA/runAaHybridVsGaN50Validation.m"],
    cwd=cwd,
    stdin=subprocess.DEVNULL,
    stdout=log,
    stderr=subprocess.STDOUT,
    start_new_session=True,
)

with open(pid_path, "w") as pid_file:
    pid_file.write(f"{process.pid}\n")

print(f"Started AA tuned hybrid N50 validation with PID {process.pid}")
print(f"Log: {log_path}")
PY
