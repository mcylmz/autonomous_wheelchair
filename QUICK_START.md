# Quick Start Guide - Refactored Wheelchair System

## Emergency Stop Usage

### Using Topic (Boolean Toggle)
```bash
# Activate emergency stop
rostopic pub /emergency_stop std_msgs/Bool "data: true"

# Release emergency stop
rostopic pub /emergency_stop std_msgs/Bool "data: false"
```

### Using Service (With Reason)
```bash
# Activate emergency stop with reason
rosservice call /trigger_emergency_stop "activate: true
reason: 'obstacle detected'"

# Release emergency stop
rosservice call /trigger_emergency_stop "activate: false
reason: ''"
```

### Monitor Emergency Stop Status
```bash
rostopic echo /emergency_stop_status
```

Output:
```yaml
active: true
reason: "manual_trigger"
timestamp:
  secs: 1738238400
  nsecs: 0
```

---

## Configuration Parameters

### ChairController Parameters
Edit launch file or use rosparam:

```xml
<node pkg="wheelchair_navigation" type="wheelchair_node" name="wheelchair_node">
    <param name="device_path" value="/dev/ttyACM0" />
    <param name="command_timeout" value="0.2" />
    <param name="monitor_rate" value="50.0" />
    <param name="command_rate" value="10.0" />
    <param name="max_retries" value="5" />
    <param name="retry_delay" value="1.0" />
</node>
```

### Motor Velocity Parameters
Load the configuration file:

```xml
<node pkg="wheelchair_navigation" type="twist2motors_refactored.py" name="twist_to_motors">
    <rosparam command="load" file="$(find wheelchair_navigation)/config/motor_params.yaml" />
</node>
```

Or set individual parameters:
```xml
<param name="wheel_diameter" value="0.380" />
<param name="gear_ratio" value="32" />
<param name="min_effective_command" value="15" />
<param name="min_motor_output" value="100" />
<param name="enable_ramping" value="true" />
<param name="max_acceleration" value="500" />
```

### FGM Planner Parameters
```xml
<node pkg="move_base" type="move_base" name="move_base">
    <param name="fgm_plugin/data_timeout_threshold" value="0.5" />
</node>
```

---

## Monitoring System Health

### Check Motor Status
```bash
rostopic echo /motor_monitor
```

Output includes:
- `encoder1`, `encoder2`: Encoder counts
- `rpm1`, `rpm2`: Motor speeds
- `amps1`, `amps2`: Motor currents
- `v_batt`: Battery voltage
- `temp1`, `temp2`: Motor temperatures
- `faultFlags`: Fault code (0 = OK)
- `mode`: Control mode (PWM/Speed/Torque)

### Warning Thresholds
The refactored system automatically warns about:
- **Battery voltage < 22V**: Warning
- **Battery voltage < 20V**: Error
- **Motor temp > 60°C**: Warning
- **Motor temp > 75°C**: Critical error
- **Fault flags ≠ 0**: Motor fault

---

## Switching Between Legacy and Refactored

### Use Refactored System (Default)
No changes needed - `wheelchair_node` is now the refactored version.

### Fallback to Legacy
Edit `CMakeLists.txt`:
```cmake
# Comment out refactored version
# add_executable(wheelchair_node src/ChairNodeRefactored.cpp ...)

# Uncomment legacy version
add_executable(wheelchair_node src/ChairNode.cpp ...)
```

Or use `wheelchair_node_legacy` executable directly in launch file:
```xml
<node pkg="wheelchair_navigation" type="wheelchair_node_legacy" name="wheelchair_node" />
```

---

## Troubleshooting

### Emergency Stop Won't Release
```bash
# Check if emergency stop is active
rostopic echo /emergency_stop_status

# Force release via service
rosservice call /trigger_emergency_stop "activate: false, reason: ''"

# Check for fault flags
rostopic echo /motor_monitor | grep faultFlags
```

### Motor Commands Are Jerky
1. Verify using refactored twist2motors:
   ```bash
   rosnode info /twist_to_motors | grep "Publications"
   # Should show twist2motors_refactored.py
   ```

2. Check ramping is enabled:
   ```bash
   rosparam get /twist_to_motors/enable_ramping
   # Should return: true
   ```

3. Adjust acceleration limit:
   ```bash
   rosparam set /twist_to_motors/max_acceleration 300
   # Lower value = smoother (but slower response)
   ```

### Sensor Data Timeout Errors
```bash
# Check data rates
rostopic hz /amcl_pose
rostopic hz /inflated_pseudo_scan
rostopic hz /scaled_lin_vel

# Should all be > 2 Hz (for 0.5s timeout)
```

Adjust timeout if needed:
```bash
rosparam set /move_base/fgm_plugin/data_timeout_threshold 1.0
```

### Device Connection Failed
1. Check device path:
   ```bash
   ls -l /dev/ttyACM*
   # Verify correct device (usually /dev/ttyACM0)
   ```

2. Check permissions:
   ```bash
   sudo chmod 666 /dev/ttyACM0
   # Or add user to dialout group:
   sudo usermod -a -G dialout $USER
   ```

3. Check retry parameters:
   ```bash
   rosparam get /wheelchair_node/max_retries
   rosparam get /wheelchair_node/retry_delay
   ```

---

## Testing the System

### Test 1: Emergency Stop Response
```bash
# Terminal 1: Monitor motor commands
rostopic echo /speed_ref

# Terminal 2: Trigger emergency stop
rostopic pub /emergency_stop std_msgs/Bool "data: true"

# Verify: speed_ref should immediately show left: 0, right: 0
```

### Test 2: Smooth Velocity Mapping
```bash
# Record motor commands
rosbag record -O velocity_test.bag /speed_ref /cmd_vel

# Send various velocities
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0, z: 0}
angular: {x: 0, y: 0, z: 0.2}"

# Stop recording (Ctrl+C) and analyze
rosbag play velocity_test.bag
rostopic echo /speed_ref > speed_commands.txt
# Plot speed_commands.txt - should be smooth curve
```

### Test 3: Sensor Timeout Recovery
```bash
# Terminal 1: Monitor planner
rostopic echo /cmd_vel

# Terminal 2: Kill sensor
rosnode kill /amcl

# Verify: /cmd_vel should show zero velocity
# Check for error: "FGM: Missing sensor data"

# Restart sensor
roslaunch ... amcl.launch

# Verify: /cmd_vel resumes normal operation
```

---

## Performance Benchmarks

| Metric | Target | How to Measure |
|--------|--------|----------------|
| Emergency stop response | < 100ms | `rostopic echo -p /emergency_stop_status` and check timestamp |
| Motor command rate | 50 Hz | `rostopic hz /motor_monitor` |
| Velocity smoothness | No jumps > 50 cmd | Plot `/speed_ref` over time |
| CPU usage | < 30% | `top -p $(pidof wheelchair_node)` |

---

## Common Parameter Tuning

### Make System More Conservative
```yaml
# Lower velocities
max_rpm: 1000  # (was 1500)
max_acceleration: 300  # (was 500)

# Stricter timeouts
command_timeout: 0.1  # (was 0.2)
data_timeout_threshold: 0.3  # (was 0.5)
```

### Make System More Responsive
```yaml
# Higher acceleration
max_acceleration: 700  # (was 500)

# Faster retry
retry_delay: 0.5  # (was 1.0)

# Higher rates
monitor_rate: 100.0  # (was 50.0)
```

### Adjust Dead-Zone Behavior
```yaml
# Wider dead-zone (less sensitive)
min_effective_command: 20  # (was 15)

# Smaller minimum output (less aggressive)
min_motor_output: 80  # (was 100)
```

---

## Safety Checklist Before Operation

- [ ] Emergency stop button is functional and accessible
- [ ] Battery voltage > 22V
- [ ] Motor temperatures < 50°C (cold start)
- [ ] No fault flags in motor monitor
- [ ] All sensors publishing at expected rates
- [ ] Operator trained on emergency stop procedure
- [ ] Clear operating area with no obstacles
- [ ] Test emergency stop response before each session

---

## Contact & Support

For issues or questions:
- Review `IMPLEMENTATION_SUMMARY.md` for detailed changes
- Check `CODE_REVIEW.md` for original analysis
- Review `CLAUDE.md` for project overview

**Emergency Procedure**: If wheelchair becomes unresponsive, immediately trigger emergency stop via `/emergency_stop` topic or power off the system.
