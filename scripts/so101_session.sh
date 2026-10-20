#!/usr/bin/env bash
# New adaptive entrypoint; modes other than --check require command review.
set -eo pipefail
SO101_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SO101_WORKSPACE/activate.sh"
set -u
export XDG_CACHE_HOME="$SO101_WORKSPACE/.cache"
export XDG_CONFIG_HOME="$SO101_WORKSPACE/.cache/config"
export XDG_DATA_HOME="$SO101_WORKSPACE/.cache/share"
export HF_HOME="$SO101_WORKSPACE/.cache/huggingface"
export HF_LEROBOT_HOME="$SO101_WORKSPACE/data/datasets"
export PYTHONUNBUFFERED=1
mkdir -p "$SO101_WORKSPACE/logs"
uv run --no-project --python "$CONDA_PREFIX/bin/python" python \
  "$SO101_WORKSPACE/scripts/so101_adaptive.py" "$@" \
  2>&1 | tee -a "$SO101_WORKSPACE/logs/so101-adaptive-session.log"
