#!/bin/zsh
set -euo pipefail

repo_dir=/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS-inffus
cd "$repo_dir"

wait_pid=""
if [[ $# -eq 2 && "$1" == "--wait-for-core-pid" ]]; then
  wait_pid="$2"
elif [[ $# -ne 0 ]]; then
  print -u2 "usage: $0 [--wait-for-core-pid PID]"
  exit 2
fi

core_completion=RUN/GA/gospa_validation/gospa_core_n50_completed.txt
pd_completion=RUN/GA/gospa_validation/gospa_pd_weighted_n50_completed.txt
external_completion=RUN/GA/reviewer_baselines_validation/reviewer_baselines_n50_completed.txt

if [[ -n "$wait_pid" ]]; then
  if [[ "$wait_pid" != <-> ]]; then
    print -u2 "invalid PID: $wait_pid"
    exit 2
  fi
  print "Waiting for active GOSPA core PID $wait_pid"
  while kill -0 "$wait_pid" 2>/dev/null; do
    sleep 30
  done
  if [[ ! -s "$core_completion" ]]; then
    print -u2 "core process ended without a completion record"
    exit 1
  fi
  # Re-open every saved seed with the latest validator and rebuild only the
  # aggregate/completion metadata. All 50 validated seed files are skipped;
  # no filtering arm is rerun.
  zsh RUN/GA/run_gospa_core_n50.sh
else
  zsh RUN/GA/run_gospa_core_n50.sh
fi

grep -qx 'trials=50' "$core_completion"
grep -qx 'gospa_ground_space=complete_extracted_kinematic_state_vector' "$core_completion"
octave --quiet --eval "addpath('RUN/GA'); d=load('RUN/GA/gospa_validation/gospa_core_n50_summary.mat','summary'); audit=validateGospaCoreAgainstLegacy(d.summary); assert(audit.localAggregateChecked); disp(audit);"

zsh RUN/GA/run_pd_weighted_gospa_n50.sh
grep -qx 'trials=50' "$pd_completion"
grep -qx 'gospa_ground_space=complete_extracted_kinematic_state_vector' "$pd_completion"
grep -qx 'legacy_per_seed_validation=passed' "$pd_completion"
grep -qx 'legacy_local_aggregate_validation=passed' "$pd_completion"
octave --quiet --eval "addpath('RUN/GA'); d=load('RUN/GA/gospa_validation/gospa_pd_weighted_n50_summary.mat','summary'); audit=validatePdWeightedAgainstLegacy(d.summary); disp(audit);"

zsh RUN/GA/run_reviewer_baselines_n50.sh
grep -qx 'trials=50' "$external_completion"
grep -qx 'gospa_ground_space=complete_extracted_kinematic_state_vector' "$external_completion"
grep -qx 'implementations=explicit_adaptations_not_exact_source_reproductions' "$external_completion"

octave --quiet --eval "addpath('RUN/GA'); paperRowsDir=fullfile(pwd,'docs','paper','els-cas-templates','sections','generated'); buildGospaMainTable([],[],[],true,fullfile(paperRowsDir,'gospa_main_table_n50_rows.tex')); buildReviewerBaselineTable([],[],[],true,fullfile(paperRowsDir,'reviewer_baseline_table_n50_network_rows.tex'),fullfile(paperRowsDir,'reviewer_baseline_table_n50_local_rows.tex'));"
octave --quiet --eval "test_reviewer_result_pipeline"

print 'REVIEWER_REVISION_VALIDATION_PIPELINE_COMPLETE'
