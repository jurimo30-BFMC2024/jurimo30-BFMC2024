#!/usr/bin/env python3
"""
Integration test for TASK 2 - Fog Controller functionality.
This test verifies that the fog handling system integrates properly
with the autoFSM and behaves correctly in various scenarios.
"""

import unittest
import sys
import os
import time
import logging
from unittest.mock import Mock, MagicMock

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

from src.core.Auto.FogController import FogController
from src.core.Auto.autoFSM import autoFSMState

class TestFogIntegration(unittest.TestCase):
    """Integration tests for fog controller within the autonomous vehicle system"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.logger = logging.getLogger('test_integration')
        self.logger.setLevel(logging.DEBUG)
        
        # Create fog controller
        self.fog_controller = FogController(512, 270, self.logger, debugging=True)
        
        # Mock queue lists for autoFSM (minimal setup)
        self.mock_queues = {
            'General': Mock(),
            'Warning': Mock(), 
            'Critical': Mock(),
            'Config': Mock()
        }
    
    def test_fog_state_enum_exists(self):
        """Test that FOG state is properly defined in autoFSM"""
        self.assertTrue(hasattr(autoFSMState, 'FOG'), "FOG state should be defined in autoFSMState")
        
    def test_fog_nodes_coverage(self):
        """Test that fog controller covers the expected node range"""
        fog_nodes = self.fog_controller.fog_nodes
        expected_nodes = {"114", "115", "116", "117", "118", "119", 
                         "122", "123", "124", "125", "126", "127", "128"}
        
        self.assertEqual(fog_nodes, expected_nodes, "Fog nodes should match expected set")
        self.assertEqual(len(fog_nodes), 13, "Should have 13 fog nodes defined")
    
    def test_fog_controller_lifecycle(self):
        """Test complete fog controller lifecycle"""
        # 1. Initially inactive
        self.assertFalse(self.fog_controller.is_active())
        
        # 2. Entering fog zone
        steering, speed = self.fog_controller.process_fog_control(
            left_x=200, right_x=300, left_visible=True, right_visible=True,
            current_node="115", normal_speed=300, front_distance=100
        )
        
        self.assertIsNotNone(steering)
        self.assertIsNotNone(speed)
        self.assertTrue(self.fog_controller.is_active())
        self.assertLess(speed, 300, "Speed should be reduced in fog")
        
        # 3. Continuing in fog zone with different conditions
        steering2, speed2 = self.fog_controller.process_fog_control(
            left_x=None, right_x=300, left_visible=False, right_visible=True,
            current_node="116", normal_speed=300, front_distance=50  # Closer obstacle
        )
        
        self.assertIsNotNone(steering2)
        self.assertIsNotNone(speed2)
        self.assertLess(speed2, speed, "Speed should reduce further with closer obstacle")
        
        # 4. Exiting fog zone
        steering3, speed3 = self.fog_controller.process_fog_control(
            left_x=200, right_x=300, left_visible=True, right_visible=True,
            current_node="200", normal_speed=300, front_distance=100
        )
        
        self.assertIsNone(steering3)
        self.assertIsNone(speed3)
        self.assertFalse(self.fog_controller.is_active())
    
    def test_fog_safety_parameters(self):
        """Test that fog controller applies appropriate safety parameters"""
        # Test speed limits
        steering, speed = self.fog_controller.process_fog_control(
            left_x=200, right_x=300, left_visible=True, right_visible=True,
            current_node="115", normal_speed=1000, front_distance=200  # Very high normal speed
        )
        
        self.assertLessEqual(speed, self.fog_controller.max_fog_speed, 
                           "Speed should not exceed maximum fog speed")
        
        # Test minimum speed enforcement
        steering, speed = self.fog_controller.process_fog_control(
            left_x=200, right_x=300, left_visible=True, right_visible=True,
            current_node="115", normal_speed=50, front_distance=10  # Very low speed and close obstacle
        )
        
        self.assertGreaterEqual(speed, self.fog_controller.min_fog_speed,
                              "Speed should not go below minimum fog speed")
        
        # Test safety distance multiplier
        multiplier = self.fog_controller.get_safety_distance_multiplier()
        self.assertEqual(multiplier, 1.5, "Safety distance should be increased by 50%")
    
    def test_fog_edge_cases(self):
        """Test fog controller behavior in edge cases"""
        # Test with no lanes visible
        steering, speed = self.fog_controller.process_fog_control(
            left_x=None, right_x=None, left_visible=False, right_visible=False,
            current_node="115", normal_speed=300, front_distance=100
        )
        
        self.assertEqual(steering, 0, "Should go straight when no lanes visible")
        self.assertIsNotNone(speed, "Should still provide speed control")
        
        # Test with very close obstacle
        steering, speed = self.fog_controller.process_fog_control(
            left_x=200, right_x=300, left_visible=True, right_visible=True,
            current_node="115", normal_speed=300, front_distance=20  # Very close
        )
        
        self.assertLess(speed, 150, "Speed should be significantly reduced for close obstacles")
    
    def test_fog_parameter_adjustment(self):
        """Test dynamic parameter adjustment functionality"""
        original_speed_factor = self.fog_controller.fog_speed_reduction_factor
        
        # Test speed in normal fog conditions
        steering1, speed1 = self.fog_controller.process_fog_control(
            left_x=200, right_x=300, left_visible=True, right_visible=True,
            current_node="115", normal_speed=300, front_distance=100
        )
        
        # Adjust parameters for more conservative driving
        self.fog_controller.update_fog_parameters(speed_reduction=0.4, safety_multiplier=2.0)
        
        # Reset and test again
        self.fog_controller.reset()
        
        steering2, speed2 = self.fog_controller.process_fog_control(
            left_x=200, right_x=300, left_visible=True, right_visible=True,
            current_node="115", normal_speed=300, front_distance=100
        )
        
        self.assertLess(speed2, speed1, "Speed should be lower with more conservative parameters")
        self.assertEqual(self.fog_controller.get_safety_distance_multiplier(), 2.0,
                        "Safety multiplier should be updated")


class TestSystemIntegration(unittest.TestCase):
    """Test integration with broader system components"""
    
    def setUp(self):
        """Set up integration test fixtures"""
        self.logger = logging.getLogger('test_system')
        self.fog_controller = FogController(512, 270, self.logger, debugging=False)
    
    def test_consistent_node_definitions(self):
        """Test that fog nodes are consistent with system expectations"""
        # Fog nodes should be strings (as used in pathPlanning)
        for node in self.fog_controller.fog_nodes:
            self.assertIsInstance(node, str, f"Node {node} should be string type")
            self.assertTrue(node.isdigit(), f"Node {node} should be numeric string")
    
    def test_speed_motor_output_format(self):
        """Test that speed output is compatible with motor control"""
        steering, speed = self.fog_controller.process_fog_control(
            left_x=200, right_x=300, left_visible=True, right_visible=True,
            current_node="115", normal_speed=300, front_distance=100
        )
        
        # Speed should be integer (compatible with motor control)
        self.assertIsInstance(speed, int, "Speed should be integer for motor control")
        self.assertGreater(speed, 0, "Speed should be positive")
        
        # Steering should be integer (compatible with motor control)  
        self.assertIsInstance(steering, int, "Steering should be integer for motor control")
    
    def test_performance_requirements(self):
        """Test that fog controller meets performance requirements"""
        start_time = time.time()
        
        # Run multiple fog control cycles
        for i in range(100):
            steering, speed = self.fog_controller.process_fog_control(
                left_x=200 + (i % 50), right_x=300 + (i % 50), 
                left_visible=True, right_visible=True,
                current_node="115", normal_speed=300, front_distance=100 - (i % 30)
            )
        
        elapsed_time = time.time() - start_time
        avg_time_per_call = elapsed_time / 100
        
        # Should process fog control in under 10ms per call for real-time performance
        self.assertLess(avg_time_per_call, 0.01, 
                       f"Fog control should be fast enough for real-time use (avg: {avg_time_per_call:.4f}s)")


def run_integration_tests():
    """Run all integration tests"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Add test cases
    suite.addTests(loader.loadTestsFromTestCase(TestFogIntegration))
    suite.addTests(loader.loadTestsFromTestCase(TestSystemIntegration))
    
    # Run tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    # Set up logging
    logging.basicConfig(level=logging.WARNING, format='%(levelname)s: %(message)s')
    
    print("Running TASK 2 Integration Tests...")
    print("Testing fog controller integration with autoFSM system...\n")
    
    success = run_integration_tests()
    
    if success:
        print("\n✅ All TASK 2 integration tests passed!")
        print("Fog handling functionality is successfully integrated!")
        exit(0)
    else:
        print("\n❌ Some integration tests failed!")
        exit(1)