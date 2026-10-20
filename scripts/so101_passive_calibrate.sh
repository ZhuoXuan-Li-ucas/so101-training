#!/usr/bin/env bash
# All --run invocations require explicit review of the displayed write contract.
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
export PYTHONDONTWRITEBYTECODE=1
exec "$CONDA_PREFIX/bin/python" "$SO101_WORKSPACE/scripts/so101_passive_calibrate.py" "$@"
