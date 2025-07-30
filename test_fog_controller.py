#!/usr/bin/env python3
"""
Unit tests for FogController module.
Tests the fog handling functionality including speed control, steering adjustments,
and safety measures for autonomous vehicle operation in foggy conditions.
"""

import unittest
import sys
import os
import numpy as np
import logging

# Add the src directory to the path to allow imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from src.core.Auto.FogController import FogController

class TestFogController(unittest.TestCase):
    """Test suite for FogController functionality"""
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        # Create a mock logger
        self.logger = logging.getLogger('test_fog')
        self.logger.setLevel(logging.DEBUG)
        
        # Initialize FogController with test parameters
        self.fog_controller = FogController(
            width=512, 
            height=270, 
            logging=self.logger, 
            debugging=True
        )
    
    def tearDown(self):
        """Clean up after each test method."""
        self.fog_controller.reset()
    
    def test_initialization(self):
        """Test that FogController initializes with correct parameters."""
        self.assertEqual(self.fog_controller.width, 512)
        self.assertEqual(self.fog_controller.height, 270)
        self.assertEqual(self.fog_controller.fog_speed_reduction_factor, 0.6)
        self.assertEqual(self.fog_controller.min_fog_speed, 80)
        self.assertEqual(self.fog_controller.max_fog_speed, 200)
        self.assertFalse(self.fog_controller.fog_active)
    
    def test_fog_zone_detection(self):
        """Test detection of fog zones."""
        # Test nodes that should be in fog zone
        self.assertTrue(self.fog_controller.is_in_fog_zone("114"))
        self.assertTrue(self.fog_controller.is_in_fog_zone("119"))
        self.assertTrue(self.fog_controller.is_in_fog_zone("128"))
        
        # Test nodes that should not be in fog zone
        self.assertFalse(self.fog_controller.is_in_fog_zone("1"))
        self.assertFalse(self.fog_controller.is_in_fog_zone("200"))
        self.assertFalse(self.fog_controller.is_in_fog_zone("999"))
    
    def test_visibility_assessment(self):
        """Test image visibility assessment."""
        # Create test images with different visibility conditions
        
        # High contrast image (good visibility)
        good_visibility_image = np.random.randint(0, 255, (270, 512), dtype=np.uint8)
        good_visibility_image[:135, :] = 50   # Dark upper half
        good_visibility_image[135:, :] = 200  # Bright lower half
        
        score = self.fog_controller.assess_visibility(good_visibility_image)
        self.assertGreater(score, 0.5, "Good visibility image should have high score")
        
        # Low contrast image (poor visibility)
        poor_visibility_image = np.full((270, 512), 128, dtype=np.uint8)  # Uniform gray
        score = self.fog_controller.assess_visibility(poor_visibility_image)
        self.assertLessEqual(score, 0.3, "Poor visibility image should have low score")
        
        # Test with None image
        score = self.fog_controller.assess_visibility(None)
        self.assertEqual(score, 0.0, "None image should return 0.0 visibility")
    
    def test_fog_speed_calculation(self):
        """Test fog speed calculation with various conditions."""
        # Test normal fog speed reduction
        normal_speed = 300
        fog_speed = self.fog_controller.calculate_fog_speed(
            normal_speed, front_distance=100, visibility_score=0.8
        )
        expected_speed = normal_speed * 0.6  # 60% reduction
        self.assertAlmostEqual(fog_speed, expected_speed, delta=10)
        
        # Test with poor visibility
        fog_speed = self.fog_controller.calculate_fog_speed(
            normal_speed, front_distance=100, visibility_score=0.3
        )
        self.assertLess(fog_speed, expected_speed, "Poor visibility should reduce speed further")
        
        # Test with close front distance
        fog_speed = self.fog_controller.calculate_fog_speed(
            normal_speed, front_distance=30, visibility_score=0.8
        )
        self.assertLess(fog_speed, expected_speed, "Close front distance should reduce speed")
        
        # Test minimum speed enforcement
        fog_speed = self.fog_controller.calculate_fog_speed(
            50, front_distance=10, visibility_score=0.1
        )
        self.assertGreaterEqual(fog_speed, self.fog_controller.min_fog_speed)
        
        # Test maximum speed enforcement
        fog_speed = self.fog_controller.calculate_fog_speed(
            1000, front_distance=200, visibility_score=1.0
        )
        self.assertLessEqual(fog_speed, self.fog_controller.max_fog_speed)
    
    def test_fog_steering_calculation(self):
        """Test fog steering calculation with various lane conditions."""
        # Test with both lanes visible
        steering = self.fog_controller.calculate_fog_steering(
            left_x=200, right_x=300, left_visible=True, right_visible=True, visibility_score=0.8
        )
        self.assertIsInstance(steering, int, "Steering should return integer")
        
        # Test with only right lane visible
        steering = self.fog_controller.calculate_fog_steering(
            left_x=None, right_x=300, left_visible=False, right_visible=True, visibility_score=0.8
        )
        self.assertIsInstance(steering, int, "Steering should work with right lane only")
        
        # Test with only left lane visible
        steering = self.fog_controller.calculate_fog_steering(
            left_x=200, right_x=None, left_visible=True, right_visible=False, visibility_score=0.8
        )
        self.assertIsInstance(steering, int, "Steering should work with left lane only")
        
        # Test with no lanes visible
        steering = self.fog_controller.calculate_fog_steering(
            left_x=None, right_x=None, left_visible=False, right_visible=False, visibility_score=0.5
        )
        self.assertEqual(steering, 0, "No lanes visible should result in straight steering")
        
        # Test reduced sensitivity with poor visibility
        steering_good = self.fog_controller.calculate_fog_steering(
            left_x=200, right_x=350, left_visible=True, right_visible=True, visibility_score=0.8
        )
        
        # Reset controller for fair comparison
        self.fog_controller.reset()
        
        steering_poor = self.fog_controller.calculate_fog_steering(
            left_x=200, right_x=350, left_visible=True, right_visible=True, visibility_score=0.3
        )
        
        # Poor visibility should result in more conservative steering
        self.assertLessEqual(abs(steering_poor), abs(steering_good), 
                           "Poor visibility should reduce steering sensitivity")
    
    def test_process_fog_control(self):
        """Test the main fog control processing function."""
        # Test outside fog zone
        steering, speed = self.fog_controller.process_fog_control(
            left_x=200, right_x=300, left_visible=True, right_visible=True,
            current_node="1", normal_speed=300, front_distance=100
        )
        self.assertIsNone(steering, "Should return None outside fog zone")
        self.assertIsNone(speed, "Should return None outside fog zone")
        
        # Test inside fog zone
        steering, speed = self.fog_controller.process_fog_control(
            left_x=200, right_x=300, left_visible=True, right_visible=True,
            current_node="115", normal_speed=300, front_distance=100
        )
        self.assertIsNotNone(steering, "Should return steering in fog zone")
        self.assertIsNotNone(speed, "Should return speed in fog zone")
        self.assertLess(speed, 300, "Fog speed should be less than normal speed")
        self.assertTrue(self.fog_controller.is_active(), "Fog controller should be active")
    
    def test_poor_visibility_tracking(self):
        """Test tracking of consecutive poor visibility frames."""
        # Simulate multiple frames with poor visibility
        for i in range(3):
            self.fog_controller.process_fog_control(
                left_x=200, right_x=300, left_visible=True, right_visible=True,
                current_node="115", normal_speed=300, front_distance=100,
                image=np.full((270, 512), 128, dtype=np.uint8)  # Poor visibility image
            )
        
        self.assertGreater(self.fog_controller.consecutive_poor_visibility_frames, 0,
                          "Should track poor visibility frames")
        
        # Test extended poor visibility speed reduction
        for i in range(10):  # Exceed threshold
            steering, speed = self.fog_controller.process_fog_control(
                left_x=200, right_x=300, left_visible=True, right_visible=True,
                current_node="115", normal_speed=300, front_distance=100,
                image=np.full((270, 512), 128, dtype=np.uint8)
            )
        
        self.assertLessEqual(speed, self.fog_controller.min_fog_speed,
                           "Extended poor visibility should trigger minimum speed")
    
    def test_controller_reset(self):
        """Test fog controller reset functionality."""
        # Activate fog controller
        self.fog_controller.process_fog_control(
            left_x=200, right_x=300, left_visible=True, right_visible=True,
            current_node="115", normal_speed=300, front_distance=100
        )
        self.assertTrue(self.fog_controller.is_active())
        
        # Reset controller
        self.fog_controller.reset()
        self.assertFalse(self.fog_controller.is_active())
        self.assertEqual(self.fog_controller.consecutive_poor_visibility_frames, 0)
    
    def test_parameter_updates(self):
        """Test dynamic parameter updates."""
        original_speed_factor = self.fog_controller.fog_speed_reduction_factor
        original_safety_mult = self.fog_controller.fog_safety_distance_multiplier
        
        # Update parameters
        self.fog_controller.update_fog_parameters(speed_reduction=0.5, safety_multiplier=2.0)
        
        self.assertEqual(self.fog_controller.fog_speed_reduction_factor, 0.5)
        self.assertEqual(self.fog_controller.fog_safety_distance_multiplier, 2.0)
        
        # Test partial update
        self.fog_controller.update_fog_parameters(speed_reduction=0.7)
        self.assertEqual(self.fog_controller.fog_speed_reduction_factor, 0.7)
        self.assertEqual(self.fog_controller.fog_safety_distance_multiplier, 2.0)  # Should remain unchanged
    
    def test_safety_distance_multiplier(self):
        """Test safety distance multiplier getter."""
        multiplier = self.fog_controller.get_safety_distance_multiplier()
        self.assertEqual(multiplier, 1.5, "Default safety distance multiplier should be 1.5")


class TestFogControllerIntegration(unittest.TestCase):
    """Integration tests for FogController with realistic scenarios"""
    
    def setUp(self):
        """Set up integration test fixtures."""
        self.logger = logging.getLogger('test_fog_integration')
        self.logger.setLevel(logging.DEBUG)
        self.fog_controller = FogController(512, 270, self.logger, debugging=False)
    
    def test_fog_zone_transition(self):
        """Test behavior when transitioning in and out of fog zones."""
        # Start outside fog zone
        result = self.fog_controller.process_fog_control(
            left_x=200, right_x=300, left_visible=True, right_visible=True,
            current_node="1", normal_speed=300, front_distance=100
        )
        self.assertEqual(result, (None, None), "Should not activate outside fog zone")
        
        # Enter fog zone
        steering, speed = self.fog_controller.process_fog_control(
            left_x=200, right_x=300, left_visible=True, right_visible=True,
            current_node="115", normal_speed=300, front_distance=100
        )
        self.assertIsNotNone(steering, "Should activate in fog zone")
        self.assertIsNotNone(speed, "Should return speed in fog zone")
        self.assertTrue(self.fog_controller.is_active(), "Should be active in fog zone")
        
        # Exit fog zone
        result = self.fog_controller.process_fog_control(
            left_x=200, right_x=300, left_visible=True, right_visible=True,
            current_node="200", normal_speed=300, front_distance=100
        )
        self.assertEqual(result, (None, None), "Should deactivate outside fog zone")
        self.assertFalse(self.fog_controller.is_active(), "Should not be active outside fog zone")
    
    def test_realistic_fog_scenario(self):
        """Test a realistic fog driving scenario."""
        # Simulate driving through fog with varying conditions
        test_scenarios = [
            # (left_x, right_x, left_vis, right_vis, node, front_dist, expected_behavior)
            (200, 300, True, True, "115", 100, "normal_fog"),      # Both lanes visible
            (None, 300, False, True, "116", 80, "right_only"),     # Right lane only
            (200, None, True, False, "117", 60, "left_only"),      # Left lane only
            (None, None, False, False, "118", 40, "no_lanes"),     # No lanes visible
            (200, 300, True, True, "119", 30, "close_obstacle"),   # Close obstacle
        ]
        
        for left_x, right_x, left_vis, right_vis, node, front_dist, scenario in test_scenarios:
            steering, speed = self.fog_controller.process_fog_control(
                left_x=left_x, right_x=right_x, left_visible=left_vis, right_visible=right_vis,
                current_node=node, normal_speed=300, front_distance=front_dist
            )
            
            # Verify reasonable outputs for each scenario
            self.assertIsNotNone(steering, f"Steering should be provided for {scenario}")
            self.assertIsNotNone(speed, f"Speed should be provided for {scenario}")
            self.assertGreaterEqual(speed, self.fog_controller.min_fog_speed, 
                                  f"Speed should be above minimum for {scenario}")
            self.assertLessEqual(speed, self.fog_controller.max_fog_speed,
                               f"Speed should be below maximum for {scenario}")
            
            # Speed should decrease with closer obstacles
            if scenario == "close_obstacle":
                self.assertLess(speed, 150, "Speed should be reduced for close obstacles")


def run_fog_tests():
    """Run all fog controller tests."""
    # Create test suite
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTests(loader.loadTestsFromTestCase(TestFogController))
    suite.addTests(loader.loadTestsFromTestCase(TestFogControllerIntegration))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    # Set up logging for tests
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    
    print("Running FogController unit tests...")
    success = run_fog_tests()
    
    if success:
        print("\n✅ All fog controller tests passed!")
        exit(0)
    else:
        print("\n❌ Some fog controller tests failed!")
        exit(1)