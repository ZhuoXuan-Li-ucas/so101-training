#!/usr/bin/env bash
# Review --check output before --execute; --execute enables torque and moves.
set -eo pipefail
SO101_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SO101_WORKSPACE/activate.sh"
set -u
export XDG_CACHE_HOME="$SO101_WORKSPACE/.cache"
export HF_HOME="$SO101_WORKSPACE/.cache/huggingface"
export HF_LEROBOT_HOME="$SO101_WORKSPACE/data/datasets"
export TORCH_HOME="$SO101_WORKSPACE/.cache/torch"
export TMPDIR="$SO101_WORKSPACE/.cache/tmp"
export HF_HUB_OFFLINE=1
export PYTHONUNBUFFERED=1
exec uv run --no-project --python "$CONDA_PREFIX/bin/python" python "$SO101_WORKSPACE/scripts/reset_follower_home.py" \
    --config="$SO101_WORKSPACE/configs/act_trial_loop_080000.proposed.json" "$@"
