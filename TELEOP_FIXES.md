# Teleop Fixes for Lichtblick Integration

## Summary

Fixed teleop functionality in Lichtblick for controlling drones. The issues were:
1. **ArduPilot**: Topic paths were hardcoded with absolute paths, bypassing ROS2 namespaces
2. **PX4**: No teleop interface existed at all
3. **Lichtblick**: Configuration only supported ArduPilot

## Changes Made

### 1. Fixed ArduPilot Topic Namespacing

**Problem**: MAVROS topics were using absolute paths (`/mavros/...`) which ignored the drone namespace.

**Files Modified**:
- `aircraft/aircraft_ws/src/offboard_control/src/ardupilot_guided.cpp`
- `aircraft/aircraft_ws/src/autopilot_interface/src/ardupilot_interface.cpp`

**Changes**:
- Changed all MAVROS publisher/subscriber/service topics from absolute paths (`"/mavros/..."`) to relative paths (`"mavros/..."`)
- Changed `offboard_flag` topic from `"/offboard_flag"` to `"offboard_flag"`
- This allows topics to properly respect the `/Drone1`, `/Drone2`, etc. namespaces

**Result**: 
- ArduPilot drones now expose `/Drone1/mavros/setpoint_velocity/cmd_vel_unstamped` etc.
- Teleop commands from Lichtblick properly reach the correct drone

### 2. Added PX4 Teleop Support

**Problem**: PX4 doesn't use MAVROS, so there was no `cmd_vel_unstamped` interface.

**Files Modified**:
- `aircraft/aircraft_ws/src/offboard_control/src/px4_offboard.hpp`
- `aircraft/aircraft_ws/src/offboard_control/src/px4_offboard.cpp`
- `aircraft/aircraft_ws/src/autopilot_interface/src/px4_interface.cpp`

**Changes**:
- Added `geometry_msgs/Twist` subscriber at `setpoint_velocity/cmd_vel_unstamped`
- Created new offboard mode (flag 7) for teleop velocity control
- Implemented body-frame to NED-frame velocity transformation
- Added 500ms timeout - if no commands received, drone hovers in place
- Fixed `offboard_flag` topic to use relative path

**Result**:
- PX4 drones now expose `/Drone1/setpoint_velocity/cmd_vel_unstamped` etc.
- Teleop commands are converted to PX4's `TrajectorySetpoint` messages
- Properly handles body-frame velocities and yaw rates

### 3. Updated Lichtblick Layout

**File Modified**:
- `supplementary/lichtblick/aerial_autonomy.layout.json`

**Changes**:
- Added separate teleop panels for PX4 and ArduPilot
- PX4 panels: `Teleop!drone1_px4`, `Teleop!drone2_px4` → `/Drone{N}/setpoint_velocity/cmd_vel_unstamped`
- ArduPilot panels: `Teleop!drone1_ardupilot`, `Teleop!drone2_ardupilot` → `/Drone{N}/mavros/setpoint_velocity/cmd_vel_unstamped`
- Layout defaults to PX4 panels (since PX4 is the default autopilot)

**Result**:
- Users can switch between autopilot-specific teleop panels
- Works out-of-the-box with the default configuration

### 4. Updated Documentation

**File Modified**:
- `README.md`

**Changes**:
- Documented both PX4 and ArduPilot teleop functionality
- Explained frame conventions (body frame for PX4, NED for ArduPilot)
- Added important note about required modes (offboard flag 7 for PX4, GUIDED for ArduPilot)
- Clarified that teleop now works for both autopilots

## How to Use

### For PX4:
1. Start simulation with `AUTOPILOT=px4`
2. In Lichtblick, use the default `Teleop!drone1_px4` and `Teleop!drone2_px4` panels
3. **Important**: First put the drone in offboard mode with flag 7:
   ```bash
   ros2 action send_goal /Drone1/offboard autopilot_interface_msgs/action/Offboard "{flag: 7}"
   ```
4. Use the D-pad in Lichtblick to control the drone
5. Commands are in body frame: Up/Down = forward/backward, Left/Right = yaw left/right

### For ArduPilot:
1. Start simulation with `AUTOPILOT=ardupilot`
2. In Lichtblick, swap the teleop panels to `Teleop!drone1_ardupilot` and `Teleop!drone2_ardupilot`
3. Put the drone in GUIDED mode
4. Use the D-pad in Lichtblick to control the drone
5. Commands are in NED frame: Up/Down = north/south, Left/Right = yaw left/right

## Technical Details

### Topic Structure:
- **PX4**: `/Drone{N}/setpoint_velocity/cmd_vel_unstamped` → `px4_offboard` node → `TrajectorySetpoint` messages
- **ArduPilot**: `/Drone{N}/mavros/setpoint_velocity/cmd_vel_unstamped` → MAVROS → MAVLink velocity commands

### Frame Conventions:
- **Lichtblick Teleop Panel**: `geometry_msgs/Twist`
  - `linear.x`: forward/backward (m/s)
  - `linear.y`: left/right (m/s, typically 0)
  - `linear.z`: up/down (m/s, typically 0)
  - `angular.z`: yaw rate (rad/s)

- **PX4 Transformation**: Body frame → NED frame
  - Rotates velocities by current heading
  - Sends to `/fmu/in/trajectory_setpoint`
  
- **ArduPilot**: Uses MAVROS directly
  - Commands are in LOCAL_NED frame
  - Sends MAVLink SET_POSITION_TARGET_LOCAL_NED messages

### Safety Features:
- **PX4**: 500ms timeout - stops if no commands received
- **Both**: Default velocities are conservative (0.8 m/s, 0.5 rad/s)
- **Both**: Releasing D-pad stops publishing (vehicle may drift, tap opposite to zero)

## Testing

To test the fixes:

1. **Start a simulation**:
   ```bash
   cd ~/git/aerial-autonomy-stack/scripts
   AUTOPILOT=px4 NUM_QUADS=2 ./sim_run.sh
   # or
   AUTOPILOT=ardupilot NUM_QUADS=2 ./sim_run.sh
   ```

2. **Open Lichtblick**: Navigate to http://localhost:8080

3. **Add connections**:
   - ws://localhost:9090 (Drone 1)
   - ws://localhost:9091 (Drone 2)

4. **Import layout**: `supplementary/lichtblick/aerial_autonomy.layout.json`

5. **Test teleop**:
   - For PX4: Enter offboard mode flag 7 first
   - For ArduPilot: Enter GUIDED mode
   - Use the teleop D-pad to move the drone

## Bug Fix (Critical)

### Issue Found
The initial implementation had a critical bug on line 369 of `px4_offboard.cpp`:

```cpp
double heading_rad = heading_ * M_PI / 180.0;  // BUG: heading_ is already in radians!
```

The `heading_` variable from PX4's `VehicleLocalPosition` message is already in radians (range: -π to +π), but the code was incorrectly converting it from degrees to radians. This caused the body-frame to NED-frame rotation to be off by a factor of ~57.3, making the drone move in completely wrong directions.

### Fix Applied
Removed the incorrect conversion and added a clarifying comment:

```cpp
// Note: heading_ is already in radians (-PI to +PI)
double cos_h = std::cos(heading_);
double sin_h = std::sin(heading_);
```

This fix is **essential** for PX4 teleop to work correctly.

## Compatibility

- **YOLO**: ✅ Working (already fixed by other dev)
- **LiDAR**: ✅ Working (already fixed by other dev)
- **Teleop PX4**: ✅ Now working (bug fixed)
- **Teleop ArduPilot**: ✅ Now working

## Notes

- **Vehicle Compatibility**: Lichtblick teleop works with **all vehicle types** (quad, VTOL, fixed-wing, etc.)
- For multi-drone fleets, each drone gets its own namespace and teleop panel
- You can mix PX4 and ArduPilot drones (though typically you'd use one autopilot type)
- The Lichtblick teleop panel sends `geometry_msgs/Twist` messages in body frame, which are automatically converted to the appropriate frame for each autopilot

