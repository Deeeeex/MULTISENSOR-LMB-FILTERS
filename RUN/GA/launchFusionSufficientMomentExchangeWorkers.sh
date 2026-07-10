#!/bin/bash
# Deterministic fixed-width launcher for the v2 confirmatory protocol.

set -u
set -o pipefail

if [ "$#" -ne 11 ]; then
    echo "launcher: expected 11 arguments" >&2
    exit 64
fi

PROJECT_ROOT=$1
BATCH_PLAN=$2
WORKER_DIR=$3
OUTPUT_DIR=$4
ARTIFACT_STEM=$5
FIRST_SEED=$6
LAST_SEED=$7
MAX_WORKERS=$8
BATCH_IDENTITY=$9
CONFIG_SHA256=${10}
ATTEMPT_DIR=${11}

if [ "$MAX_WORKERS" -ne 6 ]; then
    echo "launcher: v2 requires exactly maxWorkers=6" >&2
    exit 64
fi
if [ "$FIRST_SEED" -gt "$LAST_SEED" ]; then
    echo "launcher: invalid seed interval" >&2
    exit 64
fi
if [ ! -d "$PROJECT_ROOT" ] || [ ! -d "$WORKER_DIR" ] || \
        [ ! -d "$ATTEMPT_DIR" ] || [ ! -f "$BATCH_PLAN" ]; then
    echo "launcher: missing project, reservation, worker directory, or plan" >&2
    exit 66
fi
if ! command -v octave-cli >/dev/null 2>&1; then
    echo "launcher: octave-cli is unavailable" >&2
    exit 69
fi

TEST_LAUNCH_MODE=0
TEST_HELPER_DIR=
if [ "${FUSION_SUFFICIENT_INTERNAL_TEST_LAUNCH:-0}" = "1" ]; then
    if [[ "$ARTIFACT_STEM" != TEST_* ]] || \
            [[ "$BATCH_IDENTITY" != test-* ]] || \
            [ -z "${FUSION_SUFFICIENT_TEST_HELPER_DIR:-}" ] || \
            [ ! -d "${FUSION_SUFFICIENT_TEST_HELPER_DIR:-}" ]; then
        echo "launcher: invalid internal test launch configuration" >&2
        exit 64
    fi
    TEST_LAUNCH_MODE=1
    TEST_HELPER_DIR=$FUSION_SUFFICIENT_TEST_HELPER_DIR
fi

LIFECYCLE_CLAIM="$WORKER_DIR/.launch_claim"
EXIT_LEDGER="$WORKER_DIR/worker_exit_ledger.receipt"
CAPABILITY_PATH="$WORKER_DIR/.launcher.capability"
COMPLETION_AUTH="$WORKER_DIR/completion.authorization"
WORKER_SUCCESS="$WORKER_DIR/batch_success.receipt"
ATTEMPT_SUCCESS="$ATTEMPT_DIR/batch_success.receipt"
WORKER_FAILED="$WORKER_DIR/batch_failed.tombstone"
ATTEMPT_FAILED="$ATTEMPT_DIR/batch_failed.tombstone"
active_pids=()
active_pid_count=0
failure_reason=launcher-nonzero
success_published=0
lifecycle_claim_acquired=0
CAPABILITY_SHA256=

terminate_current_wave() {
    local pid
    if [ "$active_pid_count" -gt 0 ]; then
        for pid in "${active_pids[@]}"; do
            kill -TERM "$pid" >/dev/null 2>&1 || true
        done
        for pid in "${active_pids[@]}"; do
            wait "$pid" >/dev/null 2>&1 || true
        done
    fi
    active_pids=()
    active_pid_count=0
}

publish_failed() {
    local safe_reason plan_sha temporary_path
    safe_reason=$(printf '%s' "$failure_reason" | \
        tr '[:upper:]' '[:lower:]' | tr -c 'a-z0-9_.-' '-')
    plan_sha=unpublished
    if [ -f "$BATCH_PLAN" ]; then
        plan_sha=$(shasum -a 256 "$BATCH_PLAN" | awk '{print $1}')
    fi
    if [ ! -f "$ATTEMPT_FAILED" ]; then
        temporary_path=$(mktemp "$ATTEMPT_DIR/.batch_failed.XXXXXX") || return
        {
            printf 'failure_schema=fusion-sufficient-batch-failed-v2\n'
            printf 'batch_identity=%s\n' "$BATCH_IDENTITY"
            printf 'config_sha256=%s\n' "$CONFIG_SHA256"
            printf 'plan_sha256=%s\n' "$plan_sha"
            printf 'reason=%s\n' "$safe_reason"
        } > "$temporary_path"
        /bin/ln "$temporary_path" "$ATTEMPT_FAILED" 2>/dev/null || true
        rm -f "$temporary_path"
    fi
    if [ -f "$ATTEMPT_FAILED" ] && [ ! -f "$WORKER_FAILED" ]; then
        /bin/ln "$ATTEMPT_FAILED" "$WORKER_FAILED" 2>/dev/null || true
    fi
}

on_exit() {
    local exit_code=$?
    trap - EXIT INT TERM
    if [ "$exit_code" -ne 0 ] || [ "$success_published" -ne 1 ]; then
        if [ "$exit_code" -eq 0 ]; then
            exit_code=1
            failure_reason=missing-success-receipt
        fi
        terminate_current_wave
        if [ "$lifecycle_claim_acquired" -eq 1 ]; then
            publish_failed
        fi
    fi
    exit "$exit_code"
}

on_signal() {
    local signal_name=$1
    local signal_code=$2
    failure_reason="signal-$signal_name"
    exit "$signal_code"
}

trap on_exit EXIT
trap 'on_signal INT 130' INT
trap 'on_signal TERM 143' TERM

if [ -e "$LIFECYCLE_CLAIM" ]; then
    failure_reason=lifecycle-claim-exists
    exit 73
fi
consumed_capability="$WORKER_DIR/.launcher.capability.consumed.$$"
if ! /bin/mv "$CAPABILITY_PATH" "$consumed_capability" 2>/dev/null; then
    failure_reason=launcher-capability-unavailable
    exit 73
fi
launcher_capability=$(tr -d '\r\n' < "$consumed_capability")
CAPABILITY_SHA256=$(printf '%s' "$launcher_capability" | \
    shasum -a 256 | awk '{print $1}')
expected_capability_sha=$(awk -F= \
    '$1 == "launcher_capability_sha256" {print $2}' \
    "$ATTEMPT_DIR/reservation.receipt")
if [ -z "$launcher_capability" ] || \
        [ "$CAPABILITY_SHA256" != "$expected_capability_sha" ]; then
    rm -f "$consumed_capability"
    failure_reason=launcher-capability-mismatch
    exit 73
fi
claim_tmp=$(mktemp "$WORKER_DIR/.launch_claim.XXXXXX") || {
    rm -f "$consumed_capability"
    failure_reason=lifecycle-claim-temp-failed
    exit 73
}
{
    printf 'claim_schema=fusion-sufficient-launch-claim-v2\n'
    printf 'batch_identity=%s\n' "$BATCH_IDENTITY"
    printf 'launcher_capability_sha256=%s\n' "$CAPABILITY_SHA256"
} > "$claim_tmp"
if ! /bin/ln "$claim_tmp" "$LIFECYCLE_CLAIM" 2>/dev/null; then
    rm -f "$claim_tmp" "$consumed_capability"
    failure_reason=lifecycle-claim-exists
    exit 73
fi
rm -f "$claim_tmp" "$consumed_capability"
lifecycle_claim_acquired=1

# One batch-wide preflight after the lifecycle claim and before any worker.
shopt -s nullglob
existing_workers=("$WORKER_DIR"/worker_seed_*.mat \
    "$WORKER_DIR"/worker_seed_*.log)
if [ "${#existing_workers[@]}" -ne 0 ]; then
    failure_reason=existing-worker-artifact
    exit 73
fi
for state_path in "$WORKER_SUCCESS" "$ATTEMPT_SUCCESS" "$EXIT_LEDGER" \
        "$COMPLETION_AUTH" \
        "$WORKER_FAILED" "$ATTEMPT_FAILED"; do
    if [ -e "$state_path" ]; then
        failure_reason=existing-terminal-state
        exit 73
    fi
done
final_paths=("$OUTPUT_DIR/$ARTIFACT_STEM.mat" \
    "$OUTPUT_DIR/$ARTIFACT_STEM.csv" \
    "$OUTPUT_DIR/$ARTIFACT_STEM.md")
for final_path in "${final_paths[@]}"; do
    if [ -e "$final_path" ]; then
        failure_reason=existing-final-artifact
        exit 73
    fi
done

cd "$PROJECT_ROOT" || {
    failure_reason=project-cd-failed
    exit 72
}
PLAN_OCTAVE=$(printf '%s' "$BATCH_PLAN" | sed "s/'/''/g")
batch_start=$FIRST_SEED
while [ "$batch_start" -le "$LAST_SEED" ]; do
    pids=()
    seeds=()
    batch_end=$((batch_start + MAX_WORKERS - 1))
    if [ "$batch_end" -gt "$LAST_SEED" ]; then
        batch_end=$LAST_SEED
    fi

    seed=$batch_start
    while [ "$seed" -le "$batch_end" ]; do
        worker_mat=$(printf '%s/worker_seed_%06d.mat' "$WORKER_DIR" "$seed")
        worker_log=$(printf '%s/worker_seed_%06d.log' "$WORKER_DIR" "$seed")
        worker_mat_octave=$(printf '%s' "$worker_mat" | sed "s/'/''/g")
        echo "seed=$seed start"
        set -o noclobber
        if [ "$TEST_LAUNCH_MODE" -eq 1 ]; then
            worker_eval="setPath; addpath('RUN/GA'); t=struct('enabled',true,'skipGitProvenance',true); runFusionSufficientMomentExchangeWorker('$PLAN_OCTAVE',$seed,'$worker_mat_octave',t);"
        else
            worker_eval="setPath; addpath('RUN/GA'); runFusionSufficientMomentExchangeWorker('$PLAN_OCTAVE',$seed,'$worker_mat_octave');"
        fi
        OCTAVE_HISTFILE=/dev/null octave-cli --quiet --eval "$worker_eval" \
            > "$worker_log" 2>&1 &
        worker_pid=$!
        pids+=("$worker_pid")
        seeds+=("$seed")
        active_pids+=("$worker_pid")
        active_pid_count=$((active_pid_count + 1))
        set +o noclobber
        if [ "${FUSION_SUFFICIENT_TEST_SIGNAL_AFTER_SPAWN:-0}" = "1" ] && \
                [[ "$ARTIFACT_STEM" == TEST_* ]] && \
                [[ "$BATCH_IDENTITY" == test-* ]]; then
            kill -TERM $$
        fi
        seed=$((seed + 1))
    done

    wave_failed=0
    index=0
    while [ "$index" -lt "${#pids[@]}" ]; do
        exit_code=0
        if wait "${pids[$index]}"; then
            exit_code=0
        else
            exit_code=$?
            wave_failed=1
        fi
        unset "active_pids[$index]"
        active_pid_count=$((active_pid_count - 1))
        echo "seed=${seeds[$index]} finish exit=$exit_code"
        index=$((index + 1))
    done
    active_pids=()
    active_pid_count=0
    if [ "$wave_failed" -ne 0 ]; then
        failure_reason=worker-exit-nonzero
        exit 1
    fi
    batch_start=$((batch_end + 1))
done

worker_mats=("$WORKER_DIR"/worker_seed_*.mat)
worker_logs=("$WORKER_DIR"/worker_seed_*.log)
expected_count=$((LAST_SEED - FIRST_SEED + 1))
if [ "${#worker_mats[@]}" -ne "$expected_count" ] || \
        [ "${#worker_logs[@]}" -ne "$expected_count" ]; then
    failure_reason=incomplete-worker-artifact-set
    exit 1
fi

exit_ledger_tmp=$(mktemp "$WORKER_DIR/.exit_ledger.XXXXXX") || {
    failure_reason=exit-ledger-temp-failed
    exit 1
}
{
    printf 'exit_schema=fusion-sufficient-worker-exits-v2\n'
    printf 'batch_identity=%s\n' "$BATCH_IDENTITY"
    printf 'launcher_capability_sha256=%s\n' "$CAPABILITY_SHA256"
    printf 'ordered_seeds='
    seed=$FIRST_SEED
    while [ "$seed" -le "$LAST_SEED" ]; do
        if [ "$seed" -ne "$FIRST_SEED" ]; then
            printf ','
        fi
        printf '%d' "$seed"
        seed=$((seed + 1))
    done
    printf '\n'
    seed=$FIRST_SEED
    while [ "$seed" -le "$LAST_SEED" ]; do
        printf 'worker_%06d_exit=0\n' "$seed"
        seed=$((seed + 1))
    done
} > "$exit_ledger_tmp"
if ! /bin/ln "$exit_ledger_tmp" "$EXIT_LEDGER" 2>/dev/null; then
    rm -f "$exit_ledger_tmp"
    failure_reason=exit-ledger-publication-failed
    exit 1
fi
rm -f "$exit_ledger_tmp"

completion_token=$(openssl rand -hex 32 2>/dev/null || true)
if ! printf '%s' "$completion_token" | \
        grep -Eq '^[0-9a-f]{64}$'; then
    failure_reason=completion-token-generation-failed
    exit 1
fi
completion_token_sha=$(printf '%s' "$completion_token" | \
    shasum -a 256 | awk '{print $1}')
completion_tmp=$(mktemp "$WORKER_DIR/.completion.XXXXXX") || {
    failure_reason=completion-authorization-temp-failed
    exit 1
}
{
    printf 'completion_schema=fusion-sufficient-completion-v2\n'
    printf 'batch_identity=%s\n' "$BATCH_IDENTITY"
    printf 'launcher_capability_sha256=%s\n' "$CAPABILITY_SHA256"
    printf 'completion_token_sha256=%s\n' "$completion_token_sha"
} > "$completion_tmp"
if ! /bin/ln "$completion_tmp" "$COMPLETION_AUTH" 2>/dev/null; then
    rm -f "$completion_tmp"
    failure_reason=completion-authorization-publication-failed
    exit 1
fi
rm -f "$completion_tmp"

if [ "$TEST_LAUNCH_MODE" -eq 1 ]; then
    TEST_HELPER_OCTAVE=$(printf '%s' "$TEST_HELPER_DIR" | sed "s/'/''/g")
    success_eval="setPath; addpath('RUN/GA'); addpath('$TEST_HELPER_OCTAVE'); test_publish_fusion_sufficient_batch_success('$PLAN_OCTAVE','$completion_token','$launcher_capability');"
else
    success_eval="setPath; addpath('RUN/GA'); publishFusionSufficientBatchSuccessReceipt('$PLAN_OCTAVE','$completion_token','$launcher_capability');"
fi
if ! OCTAVE_HISTFILE=/dev/null octave-cli --quiet --eval "$success_eval" \
        >/dev/null 2>&1; then
    failure_reason=success-receipt-publication-failed
    exit 1
fi
if [ ! -f "$WORKER_SUCCESS" ] || [ ! -f "$ATTEMPT_SUCCESS" ] || \
        ! cmp -s "$WORKER_SUCCESS" "$ATTEMPT_SUCCESS" || \
        [ -f "$WORKER_FAILED" ] || [ -f "$ATTEMPT_FAILED" ]; then
    failure_reason=success-receipt-verification-failed
    exit 1
fi
success_published=1
trap - EXIT INT TERM
exit 0
