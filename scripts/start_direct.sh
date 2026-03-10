#!/bin/bash
# Direct mode: SpaceMouse → UR3 via ur_rtde (no ROS/Docker needed)
set -e
cd "$(dirname "$0")/.."

PYTHON="/opt/anaconda3/envs/sm-ur3/bin/python3"

if [ ! -f "$PYTHON" ]; then
    echo "conda env 'sm-ur3' not found. Create with:"
    echo "  conda create -n sm-ur3 python=3.12"
    echo "  conda run -n sm-ur3 pip install -r requirements.txt"
    exit 1
fi

# Kill 3DxWare if running (it locks the SpaceMouse HID device)
killall 3DconnexionHelper 3DxNLServer 3DxRadialMenu 3DxVirtualNumpad 2>/dev/null || true

$PYTHON src/teleop_direct.py --config config/teleop_config.yaml "$@"
