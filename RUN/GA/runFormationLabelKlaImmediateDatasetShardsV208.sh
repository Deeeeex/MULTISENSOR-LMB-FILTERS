#!/bin/zsh

set -eu
setopt PIPE_FAIL

script_dir="${0:A:h}"
project_root="${script_dir:h:h}"
cd "$project_root"

if (( $# < 1 )); then
    print -u2 "usage: $0 PAGE [PAGE ...]"
    exit 2
fi

evidence_root="RUN/GA/dynamic_topology/evidence/tracking_aligned_v208"
output_root="${evidence_root}/formation_label_kla_immediate_shards/x36_seed211_t72_h8"
log_root="${evidence_root}/logs"
mkdir -p "$output_root" "$log_root"

typeset -a pids
typeset -a pages
for page in "$@"; do
    if [[ "$page" != <1-8> ]]; then
        print -u2 "invalid captured-screen page: $page"
        exit 2
    fi
    page_name=$(printf 'page_%02d' "$page")
    shard_root="${output_root}/${page_name}"
    shard_log="${log_root}/formation_label_kla_immediate_${page_name}.log"
    mkdir -p "$shard_root"
    (
        octave-cli --quiet --eval "addpath(genpath(pwd)); options=struct('pageIndices',${page},'outputRoot','${shard_root}'); [reportPath,dataset]=generateFormationLabelKlaImmediateDatasetV208(options); fprintf('V208_SHARD_DONE page=${page} rows=%d joint=%d report=%s\\n',dataset.rowCount,dataset.jointAggregatePositiveRowCount,reportPath);" 2>&1 | tee "$shard_log"
    ) &
    pids+=("$!")
    pages+=("$page")
    print "V208_SHARD_STARTED page=$page pid=$! log=$shard_log"
done

exit_code=0
for index in {1..$#pids}; do
    if wait "${pids[$index]}"; then
        print "V208_SHARD_FINISHED page=${pages[$index]}"
    else
        print -u2 "V208_SHARD_FAILED page=${pages[$index]}"
        exit_code=1
    fi
done

if (( exit_code == 0 )); then
    print "V208_ALL_SHARDS_DONE pages=$*"
fi
exit "$exit_code"
