#!/usr/bin/env bash
# Display dataset video and live RGB only. Never opens a motor port.
set -euo pipefail
SO101_WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PLAYER=/home/pangk/miniforge3/envs/lerobot-so101/bin/ffplay
WRIST=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._UGREEN_Camera_2K_SN0001-video-index0
TOP=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:14.3:1.4-video-index0
REFERENCE="${1:?Usage: bash scripts/preview_wrist_alignment.sh reference-wrist.mp4}"
[[ -f "$REFERENCE" ]] || { printf 'Reference missing: %s\n' "$REFERENCE" >&2; exit 1; }
for device in "$WRIST" "$TOP"; do
    [[ -r "$device" ]] || { printf 'Camera unavailable: %s\n' "$device" >&2; exit 1; }
    if fuser -s "$device"; then
        printf 'Camera busy: %s\nClose its preview or inference process first.\n' "$device" >&2
        exit 1
    fi
done
LOG_DIR="$SO101_WORKSPACE/logs/camera-alignment-$(date +%Y%m%d-%H%M%S)-$$"
mkdir -p "$LOG_DIR"
children=()
cleanup() {
    for child in "${children[@]}"; do kill "$child" 2>/dev/null || true; done
    wait "${children[@]}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

"$PLAYER" -hide_banner -loglevel info -nostats -an -loop 0 -x 640 -y 480 -left 20 -top 40 \
    -window_title 'SO101 DATASET wrist | SPACE pause | S next frame | Q close' \
    -i "$REFERENCE" > "$LOG_DIR/reference.log" 2>&1 &
children+=("$!")

live=(-hide_banner -loglevel info -nostats -f v4l2 -input_format mjpeg
    -video_size 640x480 -framerate 30 -fflags nobuffer -flags low_delay
    -probesize 32768 -analyzeduration 0 -framedrop -sync ext -an)
"$PLAYER" "${live[@]}" -x 640 -y 480 -left 700 -top 40 \
    -window_title 'SO101 LIVE wrist | UGREEN | Q close' -i "$WRIST" \
    > "$LOG_DIR/live-wrist.log" 2>&1 &
children+=("$!")
"$PLAYER" "${live[@]}" -x 480 -y 360 -left 700 -top 590 \
    -window_title 'SO101 LIVE top | fixed Orbbec | Q close' -i "$TOP" \
    > "$LOG_DIR/live-top.log" 2>&1 &
children+=("$!")
printf 'Camera-only preview. Upper left: dataset wrist; upper right: live wrist; lower right: live top.\n'
printf 'Close all windows with Q/Esc before inference. Logs: %s\n' "$LOG_DIR"
status=0
for child in "${children[@]}"; do wait "$child" || status=$?; done
exit "$status"
