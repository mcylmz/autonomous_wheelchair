#!/usr/bin/env python
"""
Unit tests for velocity mapping in twist2motors

Tests smooth velocity mapping, dead-zone handling, and ramping.
"""

import unittest
import math


class MockTwistToMotors:
    """Mock version of TwistToMotors for testing velocity mapping logic"""

    def __init__(self):
        # Motor parameters
        self.wheel_diameter = 0.380
        self.gear_ratio = 32
        self.max_rpm = 1500
        self.max_command = 1000
        self.min_effective_command = 15
        self.min_motor_output = 100
        self.enable_ramping = True
        self.max_acceleration = 500

        # State
        self.prev_cmd_left = 0.0
        self.prev_cmd_right = 0.0

    def smooth_speed_mapping(self, raw_value):
        """Apply smooth speed mapping with dead-zone handling"""
        # Handle negative values (backward motion disabled)
        if raw_value < 0:
            return 0

        # Dead zone: below threshold means stop
        if raw_value < self.min_effective_command:
            return 0

        # Saturation: cap at maximum
        if raw_value > self.max_command:
            raw_value = self.max_command

        # Linear scaling
        scaled_value = self.min_motor_output + \
                      (raw_value - self.min_effective_command) * \
                      (self.max_command - self.min_motor_output) / \
                      (self.max_command - self.min_effective_command)

        return scaled_value

    def apply_velocity_ramping(self, target_cmd, prev_cmd, dt):
        """Apply velocity ramping to prevent sudden acceleration"""
        if not self.enable_ramping or dt <= 0:
            return target_cmd

        max_change = self.max_acceleration * dt
        desired_change = target_cmd - prev_cmd

        if abs(desired_change) > max_change:
            if desired_change > 0:
                return prev_cmd + max_change
            else:
                return prev_cmd - max_change

        return target_cmd


class TestVelocityMapping(unittest.TestCase):
    """Test cases for velocity mapping"""

    def setUp(self):
        """Set up test fixture"""
        self.mapper = MockTwistToMotors()

    def test_dead_zone_zero(self):
        """Test that values below threshold map to zero"""
        # Values below min_effective_command should map to 0
        self.assertEqual(self.mapper.smooth_speed_mapping(0), 0)
        self.assertEqual(self.mapper.smooth_speed_mapping(5), 0)
        self.assertEqual(self.mapper.smooth_speed_mapping(14), 0)
        self.assertEqual(self.mapper.smooth_speed_mapping(14.9), 0)

    def test_no_discontinuity(self):
        """Test that there's no jump at the threshold"""
        # The original bug: 15 jumped to 100
        # Now it should be smooth
        result_at_threshold = self.mapper.smooth_speed_mapping(15)

        # At threshold, should map to min_motor_output
        self.assertAlmostEqual(result_at_threshold, 100, delta=1)

        # Just above threshold should be slightly higher
        result_above = self.mapper.smooth_speed_mapping(20)
        self.assertGreater(result_above, result_at_threshold)
        self.assertLess(result_above - result_at_threshold, 10)  # Smooth increment

    def test_linear_scaling(self):
        """Test that mapping is linear in the active range"""
        # Get values at different points
        val_25 = self.mapper.smooth_speed_mapping(25)
        val_50 = self.mapper.smooth_speed_mapping(50)
        val_75 = self.mapper.smooth_speed_mapping(75)

        # Check approximately linear relationship
        diff1 = val_50 - val_25
        diff2 = val_75 - val_50

        self.assertAlmostEqual(diff1, diff2, delta=2)

    def test_saturation_at_max(self):
        """Test saturation at maximum command"""
        # At max, should return max
        self.assertEqual(self.mapper.smooth_speed_mapping(1000), 1000)

        # Above max, should still be max
        self.assertEqual(self.mapper.smooth_speed_mapping(1500), 1000)
        self.assertEqual(self.mapper.smooth_speed_mapping(2000), 1000)

    def test_negative_values_zero(self):
        """Test that negative values map to zero (no backward)"""
        self.assertEqual(self.mapper.smooth_speed_mapping(-100), 0)
        self.assertEqual(self.mapper.smooth_speed_mapping(-10), 0)
        self.assertEqual(self.mapper.smooth_speed_mapping(-1), 0)

    def test_smoothness_across_range(self):
        """Test that mapping is smooth across entire range"""
        prev_value = 0
        prev_output = self.mapper.smooth_speed_mapping(0)

        # Test increments of 10
        for value in range(0, 1001, 10):
            output = self.mapper.smooth_speed_mapping(value)

            # Output should be monotonically increasing
            self.assertGreaterEqual(output, prev_output)

            # No sudden jumps (except at dead-zone boundary)
            if value > self.mapper.min_effective_command and prev_value > 0:
                change = output - prev_output
                self.assertLess(change, 50)  # No jump > 50

            prev_value = value
            prev_output = output

    def test_velocity_ramping_no_change(self):
        """Test ramping with no change needed"""
        result = self.mapper.apply_velocity_ramping(100, 100, 0.1)
        self.assertEqual(result, 100)

    def test_velocity_ramping_small_change(self):
        """Test ramping with small change (below limit)"""
        # Change of 30 in 0.1s = 300/s < 500/s limit
        result = self.mapper.apply_velocity_ramping(130, 100, 0.1)
        self.assertAlmostEqual(result, 130, delta=1)

    def test_velocity_ramping_limited_positive(self):
        """Test ramping limits positive acceleration"""
        # Change of 100 in 0.1s = 1000/s > 500/s limit
        # Should be limited to 500 * 0.1 = 50
        result = self.mapper.apply_velocity_ramping(200, 100, 0.1)
        self.assertAlmostEqual(result, 150, delta=1)  # 100 + 50

    def test_velocity_ramping_limited_negative(self):
        """Test ramping limits negative acceleration"""
        # Change of -100 in 0.1s
        # Should be limited to -50
        result = self.mapper.apply_velocity_ramping(100, 200, 0.1)
        self.assertAlmostEqual(result, 150, delta=1)  # 200 - 50

    def test_ramping_disabled(self):
        """Test that ramping can be disabled"""
        self.mapper.enable_ramping = False

        # Large change should go through immediately
        result = self.mapper.apply_velocity_ramping(500, 0, 0.1)
        self.assertEqual(result, 500)


class TestVelocityMappingIntegration(unittest.TestCase):
    """Integration tests for complete velocity mapping pipeline"""

    def setUp(self):
        self.mapper = MockTwistToMotors()

    def test_full_pipeline_gradual_increase(self):
        """Test full pipeline with gradual speed increase"""
        commands = []
        prev_cmd = 0
        dt = 0.02  # 50 Hz

        # Simulate gradual increase
        for i in range(100):
            raw_value = i * 10  # 0, 10, 20, ..., 990
            mapped = self.mapper.smooth_speed_mapping(raw_value)
            ramped = self.mapper.apply_velocity_ramping(mapped, prev_cmd, dt)
            commands.append(ramped)
            prev_cmd = ramped

        # Verify smoothness
        for i in range(1, len(commands)):
            change = abs(commands[i] - commands[i-1])
            # Change should be limited by ramping
            max_allowed_change = self.mapper.max_acceleration * dt + 1  # +1 for rounding
            self.assertLessEqual(change, max_allowed_change,
                               f"Change too large at step {i}: {change}")

    def test_emergency_stop_response(self):
        """Test response to emergency stop (sudden deceleration)"""
        commands = []
        prev_cmd = 500  # Moving at 500
        dt = 0.02

        # Simulate emergency stop (target = 0)
        for i in range(20):
            mapped = self.mapper.smooth_speed_mapping(0)
            ramped = self.mapper.apply_velocity_ramping(mapped, prev_cmd, dt)
            commands.append(ramped)
            prev_cmd = ramped

        # Should gradually decelerate
        self.assertGreater(commands[0], 400)  # Still high initially
        self.assertLess(commands[-1], 50)    # Low at end
        self.assertEqual(commands[-1], 0)    # Eventually zero


if __name__ == '__main__':
    unittest.main()
