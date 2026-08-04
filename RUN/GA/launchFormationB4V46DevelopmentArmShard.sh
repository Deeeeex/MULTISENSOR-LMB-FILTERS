#!/bin/sh
set -eu

if [ "$#" -lt 2 ] || [ "$#" -gt 3 ]; then
    echo "usage: $0 CASE_ORDINAL ARM_ID [OUTPUT_PATH]" >&2
    exit 64
fi

case "$1" in
    ''|*[!0-9]*)
        echo "CASE_ORDINAL must be a positive integer" >&2
        exit 64
        ;;
esac
case "$2" in
    v46-repaired-reference-a70-e05|v46-repaired-sync-all-b4-e20-mc)
        ;;
    *)
        echo "ARM_ID is not one of the two frozen V46 arms" >&2
        exit 64
        ;;
esac

script_dir=$(CDPATH= cd "$(/usr/bin/dirname "$0")" && /bin/pwd -P)
repo_root=$(CDPATH= cd "$script_dir/../.." && /bin/pwd -P)
output_path=${3-}
expected_repo_root=/Users/dex/.config/superpowers/worktrees/MULTISENSOR-LMB-FILTERS/v46-development-permit
expected_git_dir=/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/.git/worktrees/v46-development-permit
expected_common_dir=/Users/dex/Desktop/Code/MULTISENSOR-LMB-FILTERS/.git
expected_pointer_sha256=3aa13c93b0e6f821363acd2337a72205843f9ea06be6c411b747c5afc9ef0ef0
launcher_path="$script_dir/launchFormationB4V46DevelopmentArmShard.sh"

if [ "$repo_root" != "$expected_repo_root" ] || \
        [ ! -f "$repo_root/.git" ] || [ -L "$repo_root/.git" ]; then
    echo "V46 launcher Git worktree identity drifted" >&2
    exit 70
fi
pointer_sha256=$(/usr/bin/shasum -a 256 "$repo_root/.git" | \
    /usr/bin/awk '{print $1}')
if [ "$pointer_sha256" != "$expected_pointer_sha256" ]; then
    echo "V46 launcher Git pointer drifted" >&2
    exit 70
fi
unsafe_git_env=$(/usr/bin/env | /usr/bin/awk -F= \
    '$1 ~ /^GIT_/ && $1 != "GIT_PAGER" {print $1; exit}')
if [ -n "$unsafe_git_env" ]; then
    echo "V46 launcher rejected Git environment override: $unsafe_git_env" >&2
    exit 70
fi
clean_git() {
    /usr/bin/env -i HOME=/Users/dex \
        PATH=/opt/homebrew/bin:/usr/bin:/bin:/usr/sbin:/sbin \
        GIT_PAGER=cat /usr/bin/git --no-replace-objects "$@"
}
actual_top_level=$(clean_git -C "$repo_root" \
    rev-parse --path-format=absolute --show-toplevel)
actual_git_dir=$(clean_git -C "$repo_root" \
    rev-parse --path-format=absolute --git-dir)
actual_common_dir=$(clean_git -C "$repo_root" \
    rev-parse --path-format=absolute --git-common-dir)
if [ "$actual_top_level" != "$expected_repo_root" ] || \
        [ "$actual_git_dir" != "$expected_git_dir" ] || \
        [ "$actual_common_dir" != "$expected_common_dir" ]; then
    echo "V46 launcher resolved a different Git identity" >&2
    exit 70
fi
untracked_executable=$(clean_git -C "$repo_root" ls-files \
    --others -- '*.m' '*.p' '*.mlx' '*.oct' '*.mex*' PKG_ADD PKG_DEL)
if [ -n "$untracked_executable" ]; then
    echo "V46 launcher rejected untracked executable source" >&2
    exit 70
fi
package_hook=$(/usr/bin/find "$repo_root" \
    \( -type f -o -type l \) \
    \( -name PKG_ADD -o -name PKG_DEL \) -print -quit)
if [ -n "$package_hook" ]; then
    echo "V46 launcher rejected an automatic path package hook" >&2
    exit 70
fi
tracked_matlab_path=$(clean_git -C "$repo_root" ls-files \
    '*.m' '*.p' '*.mlx' '*.oct' '*.mex*' | \
    /usr/bin/awk -v root="$repo_root" '
        /^tests\// || /\/private\// { next }
        {
            slash = match($0, /\/[^\/]*$/)
            if (slash == 0) print root
            else print root "/" substr($0, 1, slash - 1)
        }' | /usr/bin/sort -u | /usr/bin/paste -sd: -)
if [ -z "$tracked_matlab_path" ]; then
    echo "V46 launcher could not build the tracked MATLAB path" >&2
    exit 70
fi
if ! clean_git -C "$repo_root" diff --quiet -- || \
        ! clean_git -C "$repo_root" diff --cached --quiet --; then
    echo "V46 launcher requires a clean tracked worktree and index" >&2
    exit 70
fi
CDPATH= cd /

exec /usr/bin/env -i \
    HOME=/Users/dex \
    PATH="/opt/homebrew/bin:/usr/bin:/bin:/usr/sbin:/sbin" \
    TMPDIR=/tmp \
    GIT_PAGER=cat \
    FORMATION_B4_V46_INVOCATION_CONTRACT=formation-b4-v46-development-invocation-v1 \
    FORMATION_B4_V46_LAUNCHER_PATH="$launcher_path" \
    FORMATION_B4_V46_INIT_FILES_DISABLED=true \
    FORMATION_B4_V46_PREFLIGHT_GIT_IDENTITY_VERIFIED=true \
    FORMATION_B4_V46_PREFLIGHT_TRACKED_CLEAN=true \
    FORMATION_B4_V46_PREFLIGHT_UNTRACKED_EXECUTABLES_ABSENT=true \
    FORMATION_B4_V46_PREFLIGHT_GIT_POINTER_SHA256="$pointer_sha256" \
    FORMATION_B4_V46_REPO_ROOT="$repo_root" \
    FORMATION_B4_V46_TRACKED_MATLAB_PATH="$tracked_matlab_path" \
    FORMATION_B4_V46_CASE_ORDINAL="$1" \
    FORMATION_B4_V46_ARM_ID="$2" \
    FORMATION_B4_V46_OUTPUT_PATH="$output_path" \
    /opt/homebrew/bin/octave --quiet --no-gui --no-init-all --eval "restoredefaultpath; repoRoot=getenv('FORMATION_B4_V46_REPO_ROOT'); trackedPath=getenv('FORMATION_B4_V46_TRACKED_MATLAB_PATH'); addpath(trackedPath,'-end'); addpath(fullfile(repoRoot,'common'),'-begin'); addpath(fullfile(repoRoot,'RUN','GA'),'-begin'); runner=@runFormationB4V46DevelopmentArmShard; checker=@assertFormationB4V46DevelopmentCriticalFunctionPaths; runnerInfo=builtin('functions',runner); checkerInfo=builtin('functions',checker); expectedRunner=fullfile(repoRoot,'RUN','GA','runFormationB4V46DevelopmentArmShard.m'); expectedChecker=fullfile(repoRoot,'common','assertFormationB4V46DevelopmentCriticalFunctionPaths.m'); assert(isfield(runnerInfo,'file') && strcmp(runnerInfo.file,expectedRunner) && isfield(checkerInfo,'file') && strcmp(checkerInfo.file,expectedChecker),'FormationB4V46DevelopmentPermit:BootstrapPathShadowed','The V46 runner or Git/path checker entrypoint was shadowed.'); cd(repoRoot); caseOrdinal=str2double(getenv('FORMATION_B4_V46_CASE_ORDINAL')); armId=getenv('FORMATION_B4_V46_ARM_ID'); outputPath=getenv('FORMATION_B4_V46_OUTPUT_PATH'); if isempty(outputPath), runner(caseOrdinal,armId); else, runner(caseOrdinal,armId,outputPath); end"
