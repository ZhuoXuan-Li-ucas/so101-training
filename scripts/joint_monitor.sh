#!/usr/bin/env bash
set -eo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$WORKSPACE/activate.sh"
export XDG_CACHE_HOME="$WORKSPACE/.cache"
export XDG_CONFIG_HOME="$WORKSPACE/.cache/config"
export XDG_DATA_HOME="$WORKSPACE/.cache/share"
export PYTHONUNBUFFERED=1
exec uv run --no-project --python "$CONDA_PREFIX/bin/python" python "$WORKSPACE/scripts/joint_monitor.py" "$@"
