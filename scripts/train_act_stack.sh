#!/usr/bin/env bash
# Training only. Review configs/train_act_stack_20261015.proposed.json before starting.
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
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export HF_HUB_OFFLINE=1
export HF_DATASETS_OFFLINE=1
SO101_LOG="$SO101_WORKSPACE/logs/train-act-stack-20261015-run01.log"
if [[ -e "$SO101_LOG" || -e "$SO101_WORKSPACE/outputs/train/act_stack_20261015_run01" ]]; then
    printf '%s\n' 'Refusing to overwrite an existing training run or log.' >&2
    exit 1
fi
mkdir -p "$TMPDIR" "$SO101_WORKSPACE/logs"
"$CONDA_PREFIX/bin/python" - <<'PY'
import hashlib
import os
from pathlib import Path

path = Path(os.environ["TORCH_HOME"]) / "hub/checkpoints/resnet18-f37072fd.pth"
expected = "f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec"
with path.open("rb") as stream:
    actual = hashlib.file_digest(stream, "sha256").hexdigest()
if actual != expected:
    raise SystemExit("ResNet-18 cache checksum mismatch; training has not started.")
PY
uv run --no-project --python "$CONDA_PREFIX/bin/python" python -m lerobot.scripts.lerobot_train \
    --config_path="$SO101_WORKSPACE/configs/train_act_stack_20261015.proposed.json" \
    2>&1 | tee "$SO101_LOG"
