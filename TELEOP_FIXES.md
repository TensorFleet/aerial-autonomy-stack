# Lichtblick Teleop Setup Guide

This guide explains how to use the Lichtblick Teleoperation Panel to control drones in this stack.

## Quick Start

**IMPORTANT**: `sim_run.sh` only starts the containers. After it launches, you must manually enable teleop modes.

### Complete Workflow (PX4)

```bash
# Terminal 1: Start simulation
cd ~/aerial-autonomy-stack/scripts
AUTOPILOT=px4 NUM_QUADS=1 ./sim_run.sh

# Wait ~30 seconds for everything to boot up
# Open Lichtblick at http://localhost:8080
# Connect to ws://localhost:9090

# Terminal 2: Takeoff
ros2 action send_goal /Drone1/takeoff_action autopilot_interface_msgs/action/Takeoff "{takeoff_altitude: 40.0}"

# Terminal 3: Enable Teleop Mode (OFFBOARD VELOCITY)
ros2 action send_goal /Drone1/offboard_action autopilot_interface_msgs/action/Offboard "{offboard_setpoint_type: 3, max_duration_sec: 600.0}"
# Note: offboard_setpoint_type: 3 = VELOCITY mode, max_duration_sec: 600.0 = 10 minutes

# NOW you can use the Lichtblick D-pad to control the drone!
```

### Complete Workflow (ArduPilot)

```bash
# Terminal 1: Start simulation
cd ~/aerial-autonomy-stack/scripts
AUTOPILOT=ardupilot NUM_QUADS=1 ./sim_run.sh

# Wait ~40 seconds for ArduPilot SITL to initialize
# Open Lichtblick at http://localhost:8080
# Connect to ws://localhost:9090

# Terminal 2: Takeoff
ros2 action send_goal /Drone1/takeoff_action autopilot_interface_msgs/action/Takeoff "{takeoff_altitude: 10.0}"

# Terminal 3: Enable GUIDED Mode
ros2 service call /Drone1/mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"

# NOW you can use the Lichtblick D-pad to control the drone!
```

## Lichtblick Configuration

The layout file `supplementary/lichtblick/aerial_autonomy.layout.json` includes pre-configured teleop panels:

### PX4 Panels
- **Standard** (Forward/Yaw): `Teleop!drone1_px4`, `Teleop!drone2_px4`
  - Topic: `/Drone{N}/setpoint_velocity/cmd_vel_unstamped`
  - Up/Down = Forward/Backward, Left/Right = Yaw

- **Altitude** (Climb/Yaw): `Teleop!drone1_px4_altitude`, `Teleop!drone2_px4_altitude`
  - Topic: `/Drone{N}/setpoint_velocity/cmd_vel_unstamped`
  - Up/Down = Climb/Descend, Left/Right = Yaw

### ArduPilot Panels
- **Standard** (Forward/Yaw): `Teleop!drone1_ardupilot`, `Teleop!drone2_ardupilot`
  - Topic: `/Drone{N}/mavros/setpoint_velocity/cmd_vel_unstamped`
  - Up/Down = Forward/Backward, Left/Right = Yaw

- **Altitude** (Climb/Yaw): `Teleop!drone1_ardupilot_altitude`, `Teleop!drone2_ardupilot_altitude`
  - Topic: `/Drone{N}/mavros/setpoint_velocity/cmd_vel_unstamped`
  - Up/Down = Climb/Descend, Left/Right = Yaw

### Switching Between Panels

**Change control mode**:
1. Right-click the teleop panel
2. Select "Change panel" 
3. Choose `Teleop!drone1_px4_altitude` for altitude control

**Switch autopilot**:
1. Right-click the teleop panel
2. Select "Change panel"
3. Choose the appropriate ArduPilot panel if using ArduPilot

## Control Mapping

### Available Teleop Panels

The layout now includes **multiple teleop configurations** for different control schemes:

#### Standard Panels (Forward/Backward + Yaw)
- `Teleop!drone1_px4` / `Teleop!drone2_px4`
- `Teleop!drone1_ardupilot` / `Teleop!drone2_ardupilot`
- **Up Button**: Forward (linear.x = +0.8 m/s)
- **Down Button**: Backward (linear.x = -0.8 m/s)
- **Left Button**: Yaw left (angular.z = +0.5 rad/s)
- **Right Button**: Yaw right (angular.z = -0.5 rad/s)

#### Altitude Panels (Climb/Descend + Yaw)
- `Teleop!drone1_px4_altitude` / `Teleop!drone2_px4_altitude` (NEW!)
- `Teleop!drone1_ardupilot_altitude` / `Teleop!drone2_ardupilot_altitude` (NEW!)
- **Up Button**: Climb (linear.z = +0.8 m/s)
- **Down Button**: Descend (linear.z = -0.8 m/s)
- **Left Button**: Yaw left (angular.z = +0.5 rad/s)
- **Right Button**: Yaw right (angular.z = -0.5 rad/s)

### How to Switch Between Control Modes

**Option 1: Use Different Panels** (Recommended)
- In Lichtblick, swap between `Teleop!drone1_px4` and `Teleop!drone1_px4_altitude` panels
- Right-click the panel → "Change panel" → Select the desired teleop configuration

**Option 2: Edit Panel Settings** (Runtime)
1. Click the **⚙️ (settings icon)** on any Teleop panel
2. Configure each button to control any axis:
   - `linear-x`: Forward(+) / Backward(-)
   - `linear-y`: Left(+) / Right(-)
   - `linear-z`: Up(+) / Down(-)
   - `angular-z`: Yaw CCW(+) / Yaw CW(-)
3. Adjust velocities (default: 0.8 m/s for linear, 0.5 rad/s for angular)

### Available Control Axes

The `geometry_msgs/Twist` message supports:
- **linear.x**: Forward/backward velocity (body frame)
- **linear.y**: Left/right velocity (body frame)
- **linear.z**: Up/down velocity (body frame)
- **angular.z**: Yaw rate (body frame)

**Note**: Releasing the D-pad stops publishing commands. The drone will attempt to hover but may drift. Tap the opposite direction to stabilize.

## How It Works

### PX4 Implementation

1. `px4_offboard` node subscribes to `setpoint_velocity/cmd_vel_unstamped`
2. Receives `geometry_msgs/Twist` messages (body frame)
3. Converts to NED frame using current heading
4. Publishes `TrajectorySetpoint` to `/fmu/in/trajectory_setpoint`
5. 500ms timeout: if no commands, drone hovers

### ArduPilot Implementation

1. MAVROS natively handles `mavros/setpoint_velocity/cmd_vel_unstamped`
2. Receives `geometry_msgs/Twist` messages (body frame)
3. MAVROS converts to MAVLink `SET_POSITION_TARGET_LOCAL_NED`
4. ArduPilot processes in GUIDED mode

## Frame Conventions

### Input (Lichtblick → ROS)
- `geometry_msgs/Twist` in **FLU body frame**:
  - `linear.x`: forward (+) / backward (-) in m/s
  - `linear.y`: left (+) / right (-) in m/s (unused)
  - `linear.z`: up (+) / down (-) in m/s (unused)
  - `angular.z`: yaw rate, CCW (+) / CW (-) in rad/s

### PX4 Internal
- Converted to **NED frame** (North-East-Down)
- Rotated by current heading

### ArduPilot Internal
- MAVROS handles body-frame → LOCAL_NED conversion

## Troubleshooting

### Drone Not Responding

**PX4**:
- Check if drone is in OFFBOARD mode: `ros2 topic echo /Drone1/offboard_flag`
- Verify teleop messages are received: `ros2 topic echo /Drone1/setpoint_velocity/cmd_vel_unstamped`
- Ensure offboard action is active (max_duration not exceeded)

**ArduPilot**:
- Check if drone is in GUIDED mode: `ros2 topic echo /Drone1/mavros/state`
- Verify teleop messages are received: `ros2 topic echo /Drone1/mavros/setpoint_velocity/cmd_vel_unstamped`

### Drone Moves in Wrong Direction

- **PX4**: Check if heading is valid: `ros2 topic echo /Drone1/fmu/out/vehicle_local_position`
- **Both**: Verify control mapping in Lichtblick panel settings

### Connection Issues

- Verify Rosbridge is running on correct ports (9090, 9091, etc.)
- Check Lichtblick connection status in bottom right
- Ensure ROS topics are properly namespaced

## Safety Notes

1. **Always test in simulation first**
2. **Keep emergency stop ready**: Kill the offboard action or switch modes manually
3. **Monitor timeouts**: PX4 offboard has max_duration_sec limit
4. **Start with low velocities**: Default 0.8 m/s is conservative
5. **Indoor/GPS-denied**: Teleop works but position hold may drift without good localization

## Complete Example Session

```bash
# Terminal 1: Start simulation with sim_run.sh
cd ~/aerial-autonomy-stack/scripts
AUTOPILOT=px4 NUM_QUADS=1 ./sim_run.sh

# Wait ~30 seconds for simulation to boot up
# sim_run.sh automatically starts:
#   - Simulation container (Gazebo)
#   - Aircraft container(s) (PX4/ArduPilot + ROS2 nodes)
#   - Lichtblick container (http://localhost:8080)
#   - Rosbridge server (ws://localhost:9090)

# Open browser to http://localhost:8080
# In Lichtblick:
#   1. Click "Open connection" → Add "Rosbridge (ROS 1 & 2)" → ws://localhost:9090
#   2. Import layout: File → Import layout → ~/aerial-autonomy-stack/supplementary/lichtblick/aerial_autonomy.layout.json

# Terminal 2: Takeoff the drone
ros2 action send_goal /Drone1/takeoff_action autopilot_interface_msgs/action/Takeoff "{takeoff_altitude: 40.0}"

# Wait for takeoff to complete (~10 seconds)

# Terminal 3: Enable teleop mode (CRITICAL STEP!)
ros2 action send_goal /Drone1/offboard_action autopilot_interface_msgs/action/Offboard "{offboard_setpoint_type: 3, max_duration_sec: 600.0}"

# NOW the D-pad in Lichtblick Teleop panel will work!
# Use arrow keys to fly the drone

# When done, land:
# Terminal 4: Cancel offboard action first (Ctrl+C in Terminal 3), then land
ros2 action send_goal /Drone1/land_action autopilot_interface_msgs/action/Land "{landing_altitude: 60.0}"

# Stop everything: Press any key in Terminal 1 where sim_run.sh is running
```
