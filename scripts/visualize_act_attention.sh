#!/usr/bin/env bash
# Offline reconstruction only. No camera/motor access; no model or dataset writes.
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
export CUDA_CACHE_PATH="$SO101_WORKSPACE/.cache/cuda"
export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export HF_HUB_OFFLINE=1
export HF_DATASETS_OFFLINE=1
export WANDB_MODE=disabled
exec "$CONDA_PREFIX/bin/python" "$SO101_WORKSPACE/scripts/visualize_act_attention.py" "$@"
