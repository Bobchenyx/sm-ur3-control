#!/bin/bash
# ROS mode: Docker (ROS2 + ur_robot_driver + MoveIt Servo) + macOS SpaceMouse bridge
set -e
cd "$(dirname "$0")/.."

PYTHON="/opt/anaconda3/envs/sm-ur3/bin/python3"

if [ ! -f "$PYTHON" ]; then
    echo "conda env 'sm-ur3' not found. Create with:"
    echo "  conda create -n sm-ur3 python=3.12"
    echo "  conda run -n sm-ur3 pip install -r requirements.txt"
    exit 1
fi

# Warn if 3DxWare is running
if pgrep -f 3Dconnexion > /dev/null 2>&1; then
    echo "WARNING: 3DxWare driver is running and will block SpaceMouse access."
    echo "  To quit it: killall 3DconnexionHelper 3DxNLServer 3DxRadialMenu 3DxVirtualNumpad"
    read -p "  Kill 3DxWare now? [y/N] " answer
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        killall 3DconnexionHelper 3DxNLServer 3DxRadialMenu 3DxVirtualNumpad 2>/dev/null || true
        echo "  3DxWare killed."
    fi
fi

# Build and start Docker container
echo "Starting Docker container..."
docker compose -f docker/docker-compose.yml up -d --build

# Wait for rosbridge to be ready
echo "Waiting for rosbridge on port 9090..."
for i in $(seq 1 30); do
    if nc -z localhost 9090 2>/dev/null; then
        echo "rosbridge ready."
        break
    fi
    if [ "$i" -eq 30 ]; then
        echo "Timeout waiting for rosbridge. Check Docker logs:"
        echo "  docker compose -f docker/docker-compose.yml logs"
        exit 1
    fi
    sleep 1
done

# Start SpaceMouse bridge on macOS
echo "Starting SpaceMouse bridge..."
$PYTHON src/teleop_ros_bridge.py --config config/teleop_config.yaml "$@"

# Clean up on exit
echo "Stopping Docker container..."
docker compose -f docker/docker-compose.yml down
