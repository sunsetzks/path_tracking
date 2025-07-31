"""
Tests for Dubins path implementation
"""

import unittest
import math
import sys
import os

# Add the parent directory to the path so we can import our package
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dubins_curve.dubins import DubinsPath
from dubins_curve.types import Pose


class TestDubinsPath(unittest.TestCase):
    """Test cases for Dubins path implementation"""
    
    def setUp(self):
        """Set up test fixtures"""
        self.dubins = DubinsPath(turning_radius=1.0)
    
    def test_basic_path_calculation(self):
        """Test basic path calculation"""
        start = Pose(0, 0, 0)
        end = Pose(4, 0, 0)
        
        result = self.dubins.path(start, end)
        
        self.assertTrue(result.success)
        self.assertGreater(result.total_length, 0)
        self.assertIn(result.path_type, ['LSL', 'RSR', 'LSR', 'RSL', 'RLR', 'LRL'])
    
    def test_path_with_different_orientations(self):
        """Test path calculation with different orientations"""
        start = Pose(0, 0, math.pi/4)
        end = Pose(4, 3, math.pi/3)
        
        result = self.dubins.path(start, end)
        
        self.assertTrue(result.success)
        self.assertGreater(result.total_length, 0)
    
    def test_short_distance(self):
        """Test path calculation for very short distance"""
        start = Pose(0, 0, 0)
        end = Pose(0.1, 0.1, 0.1)
        
        result = self.dubins.path(start, end)
        
        self.assertTrue(result.success)
        self.assertGreater(result.total_length, 0)
    
    def test_zero_distance(self):
        """Test path calculation for zero distance"""
        start = Pose(0, 0, 0)
        end = Pose(0, 0, math.pi/2)
        
        result = self.dubins.path(start, end)
        
        # Should still succeed as it's just a rotation
        self.assertTrue(result.success)
        self.assertGreater(result.total_length, 0)
    
    def test_path_types_exist(self):
        """Test that all path types can be calculated"""
        start = Pose(0, 0, 0)
        end = Pose(6, 0, 0)
        
        # Test that we get a valid result
        result = self.dubins.path(start, end)
        self.assertTrue(result.success)
        
        # Test that we have segments
        self.assertGreater(len(result.segments), 0)
    
    def test_path_points_generation(self):
        """Test generation of path points"""
        start = Pose(0, 0, 0)
        end = Pose(4, 0, 0)
        
        result = self.dubins.path(start, end)
        points = self.dubins.get_path_points(result, num_points=10)
        
        self.assertTrue(result.success)
        self.assertGreater(len(points), 0)
        
        # Check that start and end points are close to expected
        self.assertAlmostEqual(points[0][0], start.x, places=2)
        self.assertAlmostEqual(points[0][1], start.y, places=2)
        self.assertAlmostEqual(points[-1][0], end.x, places=2)
        self.assertAlmostEqual(points[-1][1], end.y, places=2)
    
    def test_different_turning_radii(self):
        """Test with different turning radii"""
        start = Pose(0, 0, 0)
        end = Pose(4, 0, 0)
        
        # Test with different turning radii
        for radius in [0.5, 1.0, 2.0]:
            dubins = DubinsPath(turning_radius=radius)
            result = dubins.path(start, end)
            
            self.assertTrue(result.success)
            self.assertGreater(result.total_length, 0)
    
    def test_pose_distance_calculation(self):
        """Test pose distance calculation"""
        pose1 = Pose(0, 0, 0)
        pose2 = Pose(3, 4, 0)
        
        distance = pose1.distance_to(pose2)
        self.assertEqual(distance, 5.0)
    
    def test_pose_angle_calculation(self):
        """Test pose angle calculation"""
        pose1 = Pose(0, 0, 0)
        pose2 = Pose(1, 1, 0)
        
        angle = pose1.angle_to(pose2)
        expected_angle = math.pi/4
        self.assertAlmostEqual(angle, expected_angle, places=5)
    
    def test_pose_normalization(self):
        """Test pose angle normalization"""
        # Test angle normalization
        pose1 = Pose(0, 0, 3*math.pi)  # Should normalize to pi
        pose2 = Pose(0, 0, -math.pi/2)  # Should normalize to -pi/2
        
        self.assertAlmostEqual(pose1.theta, math.pi, places=5)
        self.assertAlmostEqual(pose2.theta, -math.pi/2, places=5)
    
    def test_path_continuity(self):
        """Test that path segments are continuous"""
        start = Pose(0, 0, 0)
        end = Pose(4, 2, math.pi/4)
        
        result = self.dubins.path(start, end)
        
        if result.success and len(result.segments) > 1:
            # Check that end of one segment matches start of next
            for i in range(len(result.segments) - 1):
                current_segment = result.segments[i]
                next_segment = result.segments[i + 1]
                
                # Check position continuity (allowing small numerical errors)
                self.assertAlmostEqual(
                    current_segment.end_pose.x, 
                    next_segment.start_pose.x, 
                    places=5
                )
                self.assertAlmostEqual(
                    current_segment.end_pose.y, 
                    next_segment.start_pose.y, 
                    places=5
                )
    
    def test_invalid_path_handling(self):
        """Test handling of potentially invalid paths"""
        # This should still work as the algorithm should find valid paths
        start = Pose(0, 0, 0)
        end = Pose(0.01, 0.01, math.pi)  # Very close but different orientation
        
        result = self.dubins.path(start, end)
        
        # The algorithm should still find a valid path
        self.assertTrue(result.success)


class TestPose(unittest.TestCase):
    """Test cases for Pose class"""
    
    def test_pose_creation(self):
        """Test pose creation"""
        pose = Pose(1.0, 2.0, math.pi/4)
        
        self.assertEqual(pose.x, 1.0)
        self.assertEqual(pose.y, 2.0)
        self.assertAlmostEqual(pose.theta, math.pi/4, places=5)
    
    def test_pose_distance(self):
        """Test pose distance calculation"""
        pose1 = Pose(0, 0, 0)
        pose2 = Pose(3, 4, 0)
        
        distance = pose1.distance_to(pose2)
        self.assertEqual(distance, 5.0)
    
    def test_pose_angle(self):
        """Test pose angle calculation"""
        pose1 = Pose(0, 0, 0)
        pose2 = Pose(1, 1, 0)
        
        angle = pose1.angle_to(pose2)
        expected_angle = math.pi/4
        self.assertAlmostEqual(angle, expected_angle, places=5)


if __name__ == '__main__':
    unittest.main()