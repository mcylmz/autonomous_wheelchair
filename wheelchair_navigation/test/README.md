# Wheelchair Navigation Test Suite

This directory contains unit and integration tests for the wheelchair navigation system.

## Test Organization

### Unit Tests (no ROS required)
- `test_chair_controller.cpp` - ChairController class tests (thread safety, emergency stop)
- `test_velocity_mapping.py` - Velocity mapping tests (smooth transitions, ramping)
- `test_pid_controller.cpp` (in fgm_plugin) - PID controller tests (saturation fix verification)

### Integration Tests (require roscore)
- `test_integration.cpp` - End-to-end system tests
- `test_data_validation.cpp` (in fgm_plugin) - Null pointer safety and data staleness

## Running Tests

### Run all tests
```bash
cd /home/murat/autonomous_wheelchair
source devel/setup.bash
catkin_make run_tests
```

### Run specific package tests
```bash
catkin_make run_tests_wheelchair_navigation
catkin_make run_tests_fgm_plugin
```

### Run specific test
```bash
rostest wheelchair_navigation test_integration.test
rosrun wheelchair_navigation test_chair_controller
```

### Run with verbose output
```bash
catkin_make run_tests_wheelchair_navigation ARGS:="-V"
```

### View test results
```bash
catkin_test_results
```

## Test Coverage

### Phase 1 Safety Fixes
- ✓ Emergency stop activation/release
- ✓ Thread safety (concurrent access)
- ✓ Velocity mapping smoothness
- ✓ Null pointer safety
- ✓ PID saturation bug fix

### Phase 2 Reliability
- ✓ Diagnostics system integration
- ✓ Configuration loading
- ✓ Error code propagation

### Phase 3 Architecture
- ✓ State machine transitions
- ✓ Multi-threaded diagnostics

## Hardware Tests

Hardware tests require actual wheelchair hardware and should be run separately:

### Connection Test
```bash
roslaunch wheelchair_navigation wheelchair_node.launch
# Verify connection to /dev/ttyACM0
```

### Emergency Stop Hardware Test
```bash
# 1. Start system
roslaunch wheelchair_navigation wheelchair.launch

# 2. Command motion
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}"

# 3. Trigger emergency stop
rosservice call /emergency_stop "activate: true
reason: 'hardware_test'"

# 4. Verify motors stopped immediately
rostopic echo /motor_monitor
```

### Stability Test (8 hours)
```bash
roslaunch wheelchair_navigation wheelchair.launch
# Monitor for crashes/faults over 8 hours
rosbag record /diagnostics /emergency_stop /motor_monitor
```

## Test Dependencies

Required packages:
- `gtest` (Google Test framework)
- `rostest` (ROS testing framework)
- Python `unittest` module

Install dependencies:
```bash
sudo apt-get install ros-kinetic-rostest libgtest-dev
```

## Continuous Integration

To integrate with CI pipeline:
```bash
# In .gitlab-ci.yml or .github/workflows/test.yml
script:
  - source devel/setup.bash
  - catkin_make run_tests
  - catkin_test_results
```

## Debugging Failed Tests

### View detailed test output
```bash
cat build/test_results/wheelchair_navigation/rostest-test_integration.xml
```

### Run test with GDB
```bash
gdb --args build/wheelchair_navigation/test_chair_controller
```

### Enable ROS logging
```bash
export ROSCONSOLE_CONFIG_FILE=/path/to/rosconsole.config
rostest wheelchair_navigation test_integration.test --text
```

## Adding New Tests

1. Create test file in appropriate package's `test/` directory
2. Add to `test/CMakeLists.txt`:
   - For unit tests: `catkin_add_gtest(test_name test_file.cpp)`
   - For integration tests: `add_rostest_gtest(test_name test_file.test test_file.cpp)`
3. Update this README with test description
4. Rebuild: `catkin_make tests`
5. Run: `catkin_make run_tests`
