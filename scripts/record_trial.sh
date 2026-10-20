#!/usr/bin/env bash
# Hardware recording must be reviewed before running (see ../AGENTS.md).
set -eo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$WORKSPACE/activate.sh"
set -u
export XDG_CACHE_HOME="$WORKSPACE/.cache"
export XDG_CONFIG_HOME="$WORKSPACE/.cache/config"
export XDG_DATA_HOME="$WORKSPACE/.cache/share"
export PYTHONUNBUFFERED=1
mkdir -p "$WORKSPACE/logs"
uv run --no-project --python "$CONDA_PREFIX/bin/python" python \
  "$WORKSPACE/scripts/record_with_preview.py" "$@" \
  2>&1 | tee -a "$WORKSPACE/logs/record-stack-white-on-red-trial.log"
