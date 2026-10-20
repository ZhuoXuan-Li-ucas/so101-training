#!/usr/bin/env bash
# Run the reviewed recording configuration with a separate log per config.
set -eo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$WORKSPACE/activate.sh"
set -u
if [[ $# -lt 2 || "$1" != --config ]]; then
  printf 'Usage: bash scripts/record_dataset.sh --config FILE [--check]\n' >&2
  exit 2
fi
RECORD_CONFIG="$2"
shift 2
RECORD_NAME="$(basename -- "$RECORD_CONFIG" .json)"
RECORD_LOG="$WORKSPACE/logs/${RECORD_NAME%.proposed}.log"
export XDG_CACHE_HOME="$WORKSPACE/.cache"
export XDG_CONFIG_HOME="$WORKSPACE/.cache/config"
export XDG_DATA_HOME="$WORKSPACE/.cache/share"
export PYTHONUNBUFFERED=1
mkdir -p "$WORKSPACE/logs"
uv run --no-project --python "$CONDA_PREFIX/bin/python" python \
  "$WORKSPACE/scripts/record_with_preview.py" --config "$RECORD_CONFIG" "$@" \
  2>&1 | tee -a "$RECORD_LOG"
