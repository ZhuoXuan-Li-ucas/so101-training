#!/usr/bin/env bash
# Live RGB preview using the existing project environment; no robot connection.
set -euo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PLAYER=/home/pangk/miniforge3/envs/lerobot-so101/bin/ffplay
UGREEN=/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._UGREEN_Camera_2K_SN0001-video-index0
# Interface 1.4 is RGB. Orbbec's by-id links can also match its depth interface.
ORBBEC=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:14.3:1.4-video-index0

filters=()
case "${1:-}" in
  "") ;;
  --grid) filters=(-vf 'drawgrid=w=iw/3:h=ih/3:t=1:c=white@0.45') ;;
  *) printf 'Usage: bash scripts/preview_cameras.sh [--grid]\n' >&2; exit 2 ;;
esac

for device in "$UGREEN" "$ORBBEC"; do
  if [[ ! -r "$device" ]]; then
    printf 'Camera unavailable: %s\nCheck /dev/v4l/by-path/ after reconnecting USB.\n' "$device" >&2
    exit 1
  fi
  if fuser -s "$device"; then
    printf 'Camera already in use: %s\nClose its current preview or recording first.\n' "$device" >&2
    exit 1
  fi
done

mkdir -p "$WORKSPACE/logs"
children=()
cleanup() {
  for child in "${children[@]}"; do kill "$child" 2>/dev/null || true; done
  wait "${children[@]}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

common=(-hide_banner -loglevel info -nostats -f v4l2 -input_format mjpeg
  -video_size 640x480 -framerate 30 -fflags nobuffer -flags low_delay
  -probesize 32768 -analyzeduration 0 -framedrop -sync ext -an
  -x 640 -y 480 -top 80)

"$PLAYER" "${common[@]}" -left 700 -window_title 'SO101 wrist | UGREEN RGB | Q: close, F: fullscreen' \
  "${filters[@]}" -i "$UGREEN" > "$WORKSPACE/logs/preview-ugreen.log" 2>&1 &
children+=("$!")
"$PLAYER" "${common[@]}" -left 20 -window_title 'SO101 top | Orbbec Gemini 335 RGB | Q: close, F: fullscreen' \
  "${filters[@]}" -i "$ORBBEC" > "$WORKSPACE/logs/preview-orbbec.log" 2>&1 &
children+=("$!")

printf 'Preview running at 640x480, requested 30 fps. Q/Esc closes each window; F toggles fullscreen.\n'
printf 'Close both windows before LeRobot recording. Logs: %s/logs/preview-*.log\n' "$WORKSPACE"
status=0
for child in "${children[@]}"; do wait "$child" || status=$?; done
exit "$status"
