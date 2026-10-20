#!/usr/bin/env bash
set -euo pipefail

source /home/pangk/workspaces/so101-training/activate.sh
export UV_HTTP_TIMEOUT="${UV_HTTP_TIMEOUT:-600}"
export UV_CONCURRENT_DOWNLOADS="${UV_CONCURRENT_DOWNLOADS:-8}"
if [[ -s "$PWD/logs/cached-requirements.txt" && -d "$PWD/.cache/wheels" ]]; then
    uv pip install --python "$CONDA_PREFIX/bin/python" --no-index --no-deps \
        --find-links "$PWD/.cache/wheels" -r "$PWD/logs/cached-requirements.txt"
fi
uv sync --project "$PWD/lerobot" --locked --inexact --no-managed-python \
    --extra feetech --extra training --extra core_scripts
