#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Set up the simulation
NUM_QUADS="${NUM_QUADS:-1}" # Number of quadcopters (default = 1)
NUM_VTOLS="${NUM_VTOLS:-0}" # Number of VTOLs (default = 0)
AUTOPILOT="${AUTOPILOT:-px4}" # Options: px4 (default), ardupilot
WORLD="${WORLD:-impalpable_greyness}" # Options: impalpable_greyness (default), apple_orchard, shibuya_crossing, swiss_town
HEADLESS="${HEADLESS:-false}" # Options: true, false (default)
CAMERA="${CAMERA:-true}" # Options: true (default), false
LIDAR="${LIDAR:-true}" # Options: true (default), false 
MODE="${MODE:-}" # Options: empty (default), dev, ...

# Detect the environment (Ubuntu/GNOME, WSL, etc.)
if command -v gnome-terminal >/dev/null 2>&1 && [ -n "$XDG_CURRENT_DESKTOP" ]; then
  DESK_ENV="gnome"
elif grep -qEi "(Microsoft|WSL)" /proc/version &> /dev/null; then
  DESK_ENV="wsl"
else
  echo "Unsupported environment" 
  exit 1
fi
echo "Desktop environment: $DESK_ENV"

# Cleanup function
cleanup() {
  DOCKER_PIDS=$(pgrep -f "docker run.*simulation-image|docker run.*aircraft-image|docker run.*lichtblick" 2>/dev/null || true)
  SIMULATION_CONTAINERS=$(docker ps -a -q --filter name=simulation-container 2>/dev/null || true)
  AIRCRAFT_CONTAINERS=$(docker ps -a -q --filter name=aircraft-container 2>/dev/null || true)
  LICHTBLICK_CONTAINERS=$(docker ps -a -q --filter name=lichtblick-container 2>/dev/null || true)
  echo "Stopping Docker containers (this will take a few seconds)..."
  if [ -n "$SIMULATION_CONTAINERS" ]; then
    docker stop $SIMULATION_CONTAINERS
  fi
  if [ -n "$AIRCRAFT_CONTAINERS" ]; then
    docker stop $AIRCRAFT_CONTAINERS
  fi
  if [ -n "$LICHTBLICK_CONTAINERS" ]; then
    docker stop $LICHTBLICK_CONTAINERS
  fi
  docker network rm aas-network 2>/dev/null && echo "Removed aas-network" || echo "Network aas-network not found or already removed"
  if [ -n "$DOCKER_PIDS" ]; then
    for dpid in $DOCKER_PIDS; do
      PARENT_PID=$(ps -o ppid= -p $dpid 2>/dev/null | tr -d ' ') # Determine process pids with a parent pid
      if [ -n "$PARENT_PID" ]; then
        echo "Killing terminal process $dpid"
        kill $dpid
      fi
    done
  fi
  echo "All-clear"
}
# Set trap to cleanup on script interruption (Ctrl+C, etc.)
trap cleanup EXIT INT TERM

# Initialize an empty variable for the flags
MODE_SIM_OPTS=""
MODE_AIR_OPTS=""
case "$MODE" in
  dev)
    # In dev mode, resources and workspaces are mounted from the host
    SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
    MODE_SIM_OPTS="--entrypoint /bin/bash"
    MODE_SIM_OPTS+=" -v ${SCRIPT_DIR}/../simulation/simulation_resources/:/simulation_resources:cached"
    MODE_SIM_OPTS+=" -v ${SCRIPT_DIR}/../simulation/simulation_ws/src:/ros2_ws/src:cached"
    MODE_AIR_OPTS="--entrypoint /bin/bash"
    MODE_AIR_OPTS+=" -v ${SCRIPT_DIR}/../aircraft/aircraft_resources/:/aircraft_resources:cached"
    MODE_AIR_OPTS+=" -v ${SCRIPT_DIR}/../aircraft/aircraft_ws/src:/ros2_ws/src:cached"
    MODE_AIR_OPTS+=" -v ${SCRIPT_DIR}/../simulation/simulation_ws/src/ground_system_msgs:/ros2_ws/src/ground_system_msgs:cached"
    ;;
  *)
    MODE_SIM_OPTS=""
    MODE_AIR_OPTS=""
    ;;
esac

# Grant access to the X server
if command -v xhost >/dev/null 2>&1; then 
  xhost +local:docker
fi

# Create network
NETWORK_NAME="aas-network"
docker network inspect "$NETWORK_NAME" >/dev/null 2>&1 || docker network create --subnet=42.42.0.0/16 "$NETWORK_NAME"

# WSL-specific options
WSL_OPTS="--env WAYLAND_DISPLAY=$WAYLAND_DISPLAY --env PULSE_SERVER=$PULSE_SERVER --volume /usr/lib/wsl:/usr/lib/wsl \
--env LD_LIBRARY_PATH=/usr/lib/wsl/lib --env LIBGL_ALWAYS_SOFTWARE=0 --env __GLX_VENDOR_LIBRARY_NAME=nvidia"

# Get primary display dimensions
get_primary_display_info() {
  local resolution=$(xrandr 2>/dev/null | grep " connected primary" | grep -oE '[0-9]+x[0-9]+' | head -1)
  if [[ ! "$resolution" =~ ^[0-9]+x[0-9]+$ ]]; then
    resolution=$(xrandr 2>/dev/null | grep " connected" | grep -oE '[0-9]+x[0-9]+' | head -1) # Fallback
  fi
  if [[ "$resolution" =~ ^[0-9]+x[0-9]+$ ]]; then
    SCREEN_WIDTH=$(echo "$resolution" | cut -d'x' -f1)
    SCREEN_HEIGHT=$(echo "$resolution" | cut -d'x' -f2)
    echo "Detected display: ${SCREEN_WIDTH}x${SCREEN_HEIGHT}"
  else
    SCREEN_WIDTH=1920
    SCREEN_HEIGHT=1080
    echo "Fallback resolution to ${SCREEN_WIDTH}x${SCREEN_HEIGHT} default"
  fi
}
get_primary_display_info

# Setup terminal dimensions
TERM_COLS=80
TERM_ROWS=32
FONT_SIZE=10
calculate_terminal_position() {
  local drone_id=$1
  SCREEN_SCALE=$((SCREEN_HEIGHT * 100 / 1080)) # Full HD = 100%
  X_POS=$(( (50 + drone_id * 50) * SCREEN_SCALE / 100 ))
  Y_POS=$(( (drone_id * 125) * SCREEN_SCALE / 100 ))
}

# Enable Shift+Ctrl+c, Shift+Ctrl+v copy-paste in xterm
XTERM_CONFIG_ARGS=(
  -xrm 'XTerm*selectToClipboard: true'
  -xrm 'XTerm*VT100.Translations: #override \
    Ctrl Shift <Key>C: copy-selection(CLIPBOARD) \n\
    Ctrl Shift <Key>V: insert-selection(CLIPBOARD)'
)

# Initialize a counter for the drone IDs
DRONE_ID=0 # 0: simulation, 1-N: drones

# Launch the simulation container
DOCKER_CMD="docker run -it --rm \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
  --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  --env ROS_DOMAIN_ID=99 --env AUTOPILOT=$AUTOPILOT --env DRONE_TYPE=$DRONE_TYPE \
  --env NUM_QUADS=$NUM_QUADS --env NUM_VTOLS=$NUM_VTOLS \
  --env WORLD=$WORLD --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
  --env SIMULATED_TIME=true \
  --net=aas-network --ip=42.42.1.99 \
  --privileged \
  --name simulation-container"
# Add WSL-specific options and complete the command
if [[ "$DESK_ENV" == "wsl" ]]; then
  DOCKER_CMD="$DOCKER_CMD $WSL_OPTS"
fi
DOCKER_CMD="$DOCKER_CMD ${MODE_SIM_OPTS} simulation-image"
calculate_terminal_position $DRONE_ID
xterm "${XTERM_CONFIG_ARGS[@]}" -title "Simulation" -fa Monospace -fs $FONT_SIZE -bg black -fg white -geometry "${TERM_COLS}x${TERM_ROWS}+${X_POS}+${Y_POS}" -hold -e bash -c "$DOCKER_CMD" &

# Launch the quad containers
for i in $(seq 1 $NUM_QUADS); do
  DRONE_ID=$((DRONE_ID + 1))
  ROSBRIDGE_PORT=$((9090 + DRONE_ID - 1))
  sleep 1.5 # Limit resource usage
  DOCKER_CMD="docker run -it --rm \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
    --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    --env ROS_DOMAIN_ID=$DRONE_ID --env AUTOPILOT=$AUTOPILOT \
    --env DRONE_TYPE=quad \
    --env DRONE_ID=$DRONE_ID --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
    --env SIMULATED_TIME=true \
    --net=aas-network --ip=42.42.1.$DRONE_ID \
    --privileged \
    -p ${ROSBRIDGE_PORT}:9090 \
    --name aircraft-container_$DRONE_ID"
  # Add WSL-specific options and complete the command
  if [[ "$DESK_ENV" == "wsl" ]]; then
    DOCKER_CMD="$DOCKER_CMD $WSL_OPTS"
  fi
  DOCKER_CMD="$DOCKER_CMD ${MODE_AIR_OPTS} aircraft-image"
  calculate_terminal_position $DRONE_ID
  xterm "${XTERM_CONFIG_ARGS[@]}" -title "Quad $DRONE_ID" -fa Monospace -fs $FONT_SIZE -bg black -fg white -geometry "${TERM_COLS}x${TERM_ROWS}+${X_POS}+${Y_POS}" -hold -e bash -c "$DOCKER_CMD" &
done

# Launch the vtol containers
for i in $(seq 1 $NUM_VTOLS); do
  DRONE_ID=$((DRONE_ID + 1))
  ROSBRIDGE_PORT=$((9090 + DRONE_ID - 1))
  sleep 1.5 # Limit resource usage
  DOCKER_CMD="docker run -it --rm \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw --device /dev/dri --gpus all \
    --env DISPLAY=$DISPLAY --env QT_X11_NO_MITSHM=1 --env NVIDIA_DRIVER_CAPABILITIES=all --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    --env ROS_DOMAIN_ID=$DRONE_ID --env AUTOPILOT=$AUTOPILOT \
    --env DRONE_TYPE=vtol \
    --env DRONE_ID=$DRONE_ID --env HEADLESS=$HEADLESS --env CAMERA=$CAMERA --env LIDAR=$LIDAR \
    --env SIMULATED_TIME=true \
    --net=aas-network --ip=42.42.1.$DRONE_ID \
    --privileged \
    -p ${ROSBRIDGE_PORT}:9090 \
    --name aircraft-container_$DRONE_ID"
  # Add WSL-specific options and complete the command
  if [[ "$DESK_ENV" == "wsl" ]]; then
    DOCKER_CMD="$DOCKER_CMD $WSL_OPTS"
  fi
  DOCKER_CMD="$DOCKER_CMD ${MODE_AIR_OPTS} aircraft-image"
  calculate_terminal_position $DRONE_ID
  xterm "${XTERM_CONFIG_ARGS[@]}" -title "VTOL $DRONE_ID" -fa Monospace -fs $FONT_SIZE -bg black -fg white -geometry "${TERM_COLS}x${TERM_ROWS}+${X_POS}+${Y_POS}" -hold -e bash -c "$DOCKER_CMD" &
done

if [[ "$HEADLESS" != "true" ]]; then
  LICHTBLICK_PORT="${LICHTBLICK_PORT:-8080}"
  if ! docker ps --format '{{.Names}}' | grep -q '^lichtblick-container$'; then
    echo "Starting Lichtblick visualization on http://localhost:${LICHTBLICK_PORT}"
    docker run --rm -d \
      -p "${LICHTBLICK_PORT}:8080" \
      --name lichtblick-container \
      ghcr.io/lichtblick-suite/lichtblick:latest >/dev/null
  else
    echo "Lichtblick visualization container already running."
  fi
fi

echo "Fly, my pretties, fly!"
echo "Press any key to stop all containers and close the terminals..."
read -n 1 -s # Wait for user input
