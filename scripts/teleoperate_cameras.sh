#!/usr/bin/env bash
# Start the installed LeRobot teleoperation command with this rig's configuration.
set -eo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$WORKSPACE/activate.sh"
set -u
TELEOP_CONFIG="$WORKSPACE/configs/teleoperate_cameras.json"
if [[ "${1:-}" == --config ]]; then
  if [[ $# -lt 2 ]]; then
    printf 'Usage: bash scripts/teleoperate_cameras.sh [--config FILE] [LeRobot options]\n' >&2
    exit 2
  fi
  TELEOP_CONFIG="$2"
  shift 2
fi

export XDG_CACHE_HOME="$WORKSPACE/.cache"
export XDG_CONFIG_HOME="$WORKSPACE/.cache/config"
export XDG_DATA_HOME="$WORKSPACE/.cache/share"
export PYTHONUNBUFFERED=1
export LEROBOT_RERUN_MEMORY_LIMIT=256MB

# Verify existing calibration and fault flags without running calibration. Seed
# the follower's current position before stock configure() re-enables torque,
# so an old Goal_Position cannot cause a jump during startup.
uv run --no-project --python "$CONDA_PREFIX/bin/python" python - "$WORKSPACE" "$TELEOP_CONFIG" "$@" <<'PY'
import sys
from pathlib import Path

import draccus

root = Path(sys.argv[1])
sys.path.insert(0, str(root / 'scripts'))
from diagnose_teleop import ensure_port_free, feedback
from lerobot.scripts.lerobot_teleoperate import TeleoperateConfig
from lerobot.robots import make_robot_from_config
from lerobot.teleoperators import make_teleoperator_from_config

cfg = draccus.parse(TeleoperateConfig, config_path=sys.argv[2], args=sys.argv[3:])
for port in (cfg.teleop.port, cfg.robot.port):
    ensure_port_free(port)
for camera in cfg.robot.cameras.values():
    ensure_port_free(str(camera.index_or_path))

leader = make_teleoperator_from_config(cfg.teleop)
follower = make_robot_from_config(cfg.robot)
arms = [('leader', leader), ('follower', follower)]
try:
    for role, arm in arms:
        if len(arm.calibration) != 6:
            raise RuntimeError(f'{role}: missing six-joint calibration at {arm.calibration_fpath}')
        arm.bus.connect()
        if not arm.bus.is_calibrated:
            raise RuntimeError(f'{role}: motor calibration differs from {arm.calibration_fpath}; stopping')
        values = feedback(arm.bus)
        faults = {name: v for name, v in values.items() if v['Status'] or v['packet_error']}
        if faults:
            raise RuntimeError(f'{role}: motor fault: {faults}')
        volts = [v['Present_Voltage'] / 10 for v in values.values()]
        print(f'{role}: calibration matches; all 6 motors respond without faults; voltage {min(volts):.1f}-{max(volts):.1f} V')
    current = follower.bus.sync_read('Present_Position', normalize=False)
    follower.bus.sync_write('Goal_Position', current, normalize=False)
    print('Follower startup target set to its current measured position.')
finally:
    for _, arm in arms:
        if arm.bus.is_connected:
            arm.bus.disconnect(disable_torque=False)
PY

printf '\nStarting teleoperation with the configuration and overrides printed below. Ctrl+C stops control.\n'
exec uv run --no-project --python "$CONDA_PREFIX/bin/python" lerobot-teleoperate \
  --config_path="$TELEOP_CONFIG" "$@"
