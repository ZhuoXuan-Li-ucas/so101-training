#!/usr/bin/env bash
# Offline evaluation only: no robot/camera connections or model updates.
set -eo pipefail
SO101_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$SO101_WORKSPACE/activate.sh"
set -u
export XDG_CACHE_HOME="$SO101_WORKSPACE/.cache"
export XDG_CONFIG_HOME="$SO101_WORKSPACE/.cache/config"
export XDG_DATA_HOME="$SO101_WORKSPACE/.cache/share"
export HF_HOME="$SO101_WORKSPACE/.cache/huggingface"
export HF_DATASETS_CACHE="$SO101_WORKSPACE/.cache/huggingface/datasets"
export HF_LEROBOT_HOME="$SO101_WORKSPACE/data/datasets"
export TORCH_HOME="$SO101_WORKSPACE/.cache/torch"
export TMPDIR="$SO101_WORKSPACE/.cache/tmp"
export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export HF_HUB_OFFLINE=1
export HF_DATASETS_OFFLINE=1
export WANDB_MODE=disabled
exec "$CONDA_PREFIX/bin/python" "$SO101_WORKSPACE/scripts/evaluate_act_full.py" \
    --config "$SO101_WORKSPACE/configs/eval_act_stack_full_20261017.proposed.json" "$@"
