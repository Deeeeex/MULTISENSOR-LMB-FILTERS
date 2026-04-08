#!/bin/zsh
set -euo pipefail

cd "$(dirname "$0")"
tectonic --keep-logs --keep-intermediates manuscript.tex
