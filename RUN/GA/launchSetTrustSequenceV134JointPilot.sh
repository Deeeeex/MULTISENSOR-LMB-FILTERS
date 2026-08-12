#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
V134_ROOT="${V134_ROOT:-$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel)}"
BASELINE_BRANCH="${V133_BASELINE_BRANCH:-refs/heads/codex/v70-local-scale-consistent}"
BASELINE_ROOT="${V133_BASELINE_ROOT:-}"
EXPECTED_BASELINE_COMMIT="${V133_BASELINE_COMMIT:-b207115f9010af6c4dac067e34efbdb27a2de1d1}"
POLL_SECONDS="${V134_HANDOFF_POLL_SECONDS:-30}"

if [[ -z "$BASELINE_ROOT" ]]; then
    BASELINE_ROOT="$(git -C "$V134_ROOT" worktree list --porcelain | awk \
        -v target="$BASELINE_BRANCH" '
            $1 == "worktree" { path = $2 }
            $1 == "branch" && $2 == target { print path; exit }
        ')"
fi
if [[ -z "$BASELINE_ROOT" || ! -d "$BASELINE_ROOT" ]]; then
    echo "Could not locate the frozen V133 baseline worktree." >&2
    exit 2
fi
if [[ ! "$POLL_SECONDS" =~ ^[1-9][0-9]*$ ]]; then
    echo "V134_HANDOFF_POLL_SECONDS must be a positive integer." >&2
    exit 2
fi

BASELINE_COMMIT="$(git -C "$BASELINE_ROOT" rev-parse HEAD)"
if [[ "$BASELINE_COMMIT" != "$EXPECTED_BASELINE_COMMIT" ]]; then
    echo "V133 baseline commit drift: $BASELINE_COMMIT" >&2
    exit 2
fi

V134_COMMIT="$(git -C "$V134_ROOT" rev-parse HEAD)"
if [[ -n "$(git -C "$V134_ROOT" status --porcelain --untracked-files=no)" ]]; then
    echo "V134 tracked source is dirty at launch." >&2
    exit 2
fi
if [[ -n "$(git -C "$V134_ROOT" ls-files --others --exclude-standard -- ':(glob)**/*.m')" ]]; then
    echo "V134 has untracked MATLAB/Octave source at launch." >&2
    exit 2
fi

BASELINE_OUTPUT="$BASELINE_ROOT/RUN/GA/dynamic_topology/evidence/tracking_aligned_v133/counterfactual_regret_gate/baseline_selection"
BASELINE_SHARDS="$BASELINE_OUTPUT/shards"
BASELINE_ARTIFACT="$BASELINE_OUTPUT/FROZEN_REFERENCE_CARRIER_V133.mat"
LOG_ROOT="$V134_ROOT/RUN/GA/dynamic_topology/evidence/tracking_aligned_v134/binary_admission_sequence_v4/logs"
mkdir -p "$LOG_ROOT"

LOCK_DIR="$LOG_ROOT/.joint_pilot_handoff.lock"
if ! mkdir "$LOCK_DIR" 2>/dev/null; then
    echo "Another V134 joint-pilot handoff is already active: $LOCK_DIR" >&2
    exit 3
fi
cleanup() {
    rmdir "$LOCK_DIR" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

stamp() {
    date '+%Y-%m-%d %H:%M:%S'
}

echo "[$(stamp)] V134 handoff source: $V134_COMMIT"
echo "[$(stamp)] Frozen V133 source: $BASELINE_COMMIT"

while true; do
    shard_count="$(find "$BASELINE_SHARDS" -type f -name '*.mat' 2>/dev/null | wc -l | tr -d ' ')"
    if [[ "$shard_count" == "16" ]]; then
        break
    fi
    if (( shard_count > 16 )); then
        echo "Unexpected V133 shard count: $shard_count" >&2
        exit 4
    fi
    if ! pgrep -f 'runCounterfactualRegretGateV133BaselineWorker' >/dev/null; then
        echo "V133 workers stopped with only $shard_count/16 shards." >&2
        exit 4
    fi
    echo "[$(stamp)] Waiting for frozen static carriers: $shard_count/16 shards"
    sleep "$POLL_SECONDS"
done

echo "[$(stamp)] Finalizing the frozen V133 carrier selection"
(
    cd "$BASELINE_ROOT"
    octave --quiet --eval \
        "addpath(genpath(pwd)); finalizeCounterfactualRegretGateV133BaselineSelection();" \
        2>&1 | tee "$LOG_ROOT/finalize_v133_carriers.log"
)
if [[ ! -f "$BASELINE_ARTIFACT" ]]; then
    echo "The frozen V133 carrier artifact was not created." >&2
    exit 5
fi

run_prepare() {
    local preset="$1"
    local log_name="$2"
    (
        cd "$V134_ROOT"
        octave --quiet --eval \
            "addpath(genpath(pwd)); bp='$BASELINE_ARTIFACT'; runSetTrustSequenceV134Pilot('$preset', struct('mode','prepare','baselineSelectionPath',bp));" \
            2>&1 | tee "$LOG_ROOT/$log_name"
    )
}

echo "[$(stamp)] Preparing paired M24 and X36 pilot states"
run_prepare 'm24-formation-fov' 'prepare_m24.log' &
prepare_m24_pid=$!
run_prepare 'x36-formation-fov' 'prepare_x36.log' &
prepare_x36_pid=$!
prepare_status=0
wait "$prepare_m24_pid" || prepare_status=$?
wait "$prepare_x36_pid" || prepare_status=$?
if (( prepare_status != 0 )); then
    echo "At least one V134 prepare stage failed." >&2
    exit "$prepare_status"
fi

run_worker() {
    local preset="$1"
    local worker_index="$2"
    local worker_count="$3"
    local log_name="$4"
    (
        cd "$V134_ROOT"
        octave --quiet --eval \
            "addpath(genpath(pwd)); runSetTrustSequenceV134Pilot('$preset', struct('mode','worker','workerIndex',$worker_index,'workerCount',$worker_count));" \
            2>&1 | tee "$LOG_ROOT/$log_name"
    )
}

# One M24 worker and three X36 workers approximately balance wall-clock load:
# M24 evaluates 12 actions of 53 pages; X36 evaluates 25 actions of 81 pages.
echo "[$(stamp)] Evaluating the frozen V134 action banks"
run_worker 'm24-formation-fov' 1 1 'worker_m24_1of1.log' &
worker_pids=("$!")
for worker_index in 1 2 3; do
    run_worker 'x36-formation-fov' "$worker_index" 3 \
        "worker_x36_${worker_index}of3.log" &
    worker_pids+=("$!")
done
worker_status=0
for pid in "${worker_pids[@]}"; do
    wait "$pid" || worker_status=$?
done
if (( worker_status != 0 )); then
    echo "At least one V134 action worker failed." >&2
    exit "$worker_status"
fi

echo "[$(stamp)] Finalizing scale gates and the joint gate"
(
    cd "$V134_ROOT"
    octave --quiet --eval "addpath(genpath(pwd)); runSetTrustSequenceV134Pilot('m24-formation-fov', struct('mode','finalize'));" \
        2>&1 | tee "$LOG_ROOT/finalize_m24.log"
    octave --quiet --eval "addpath(genpath(pwd)); runSetTrustSequenceV134Pilot('x36-formation-fov', struct('mode','finalize'));" \
        2>&1 | tee "$LOG_ROOT/finalize_x36.log"
    octave --quiet --eval "addpath(genpath(pwd)); finalizeSetTrustSequenceV134JointGate();" \
        2>&1 | tee "$LOG_ROOT/finalize_joint_gate.log"
)

echo "[$(stamp)] V134 joint-pilot pipeline completed"
