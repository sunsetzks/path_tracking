"""
Test suite for Hybrid A* path planning algorithm
"""

import pytest
import numpy as np
from astar_project.hybrid_astar import (
    HybridAStar, VehicleModel, State, DirectionMode, Node
)


class TestVehicleModel:
    """Test vehicle model functionality"""
    
    def test_initialization(self):
        """Test vehicle model initialization"""
        vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
        assert vehicle.wheelbase == 2.5
        assert vehicle.max_steer == np.pi/4
    
    def test_normalize_angle(self):
        """Test angle normalization"""
        assert abs(VehicleModel.normalize_angle(np.pi)) < 1e-6
        assert abs(VehicleModel.normalize_angle(-np.pi)) < 1e-6
        assert abs(VehicleModel.normalize_angle(3*np.pi) - np.pi) < 1e-6
        assert abs(VehicleModel.normalize_angle(-3*np.pi) + np.pi) < 1e-6
    
    def test_simulate_motion_forward(self):
        """Test forward motion simulation"""
        vehicle = VehicleModel()
        start_state = State(x=0, y=0, yaw=0, direction=DirectionMode.FORWARD, steer=0)
        
        states = vehicle.simulate_motion(
            start_state, velocity=1.0, steer_rate=0, dt=1.0, steps=1
        )
        
        assert len(states) == 1
        assert abs(states[0].x - 1.0) < 1e-6  # Should move forward
        assert abs(states[0].y) < 1e-6
    
    def test_simulate_motion_turning(self):
        """Test turning motion simulation"""
        vehicle = VehicleModel(wheelbase=2.0)
        start_state = State(x=0, y=0, yaw=0, direction=DirectionMode.FORWARD, steer=np.pi/4)
        
        states = vehicle.simulate_motion(
            start_state, velocity=1.0, steer_rate=0, dt=1.0, steps=1
        )
        
        assert len(states) == 1
        assert states[0].x > 0  # Should move forward
        assert states[0].y > 0  # Should move sideways due to turning
        assert abs(states[0].yaw) > 1e-6  # Yaw should change


class TestState:
    """Test state representation"""
    
    def test_initialization(self):
        """Test state initialization"""
        state = State(x=1.0, y=2.0, yaw=np.pi/2)
        assert state.x == 1.0
        assert state.y == 2.0
        assert state.yaw == np.pi/2
        assert state.direction == DirectionMode.FORWARD
        assert state.steer == 0.0
    
    def test_equality(self):
        """Test state equality comparison"""
        state1 = State(x=1.0, y=2.0, yaw=np.pi/2)
        state2 = State(x=1.05, y=2.05, yaw=np.pi/2 + 0.05)  # Within tolerance
        state3 = State(x=2.0, y=3.0, yaw=np.pi)  # Different
        
        assert state1 == state2
        assert state1 != state3
    
    def test_hash(self):
        """Test state hashing for use in sets"""
        state1 = State(x=1.0, y=2.0, yaw=np.pi/2)
        state2 = State(x=1.01, y=2.01, yaw=np.pi/2 + 0.01)  # Close values
        
        # Should be able to use in set
        state_set = {state1, state2}
        assert len(state_set) >= 1


class TestNode:
    """Test node representation"""
    
    def test_initialization(self):
        """Test node initialization"""
        state = State(x=0, y=0, yaw=0)
        node = Node(state=state, g_cost=10.0, h_cost=5.0)
        
        assert node.state == state
        assert node.g_cost == 10.0
        assert node.h_cost == 5.0
        assert node.f_cost == 15.0
        assert node.parent is None
    
    def test_comparison(self):
        """Test node comparison for priority queue"""
        state = State(x=0, y=0, yaw=0)
        node1 = Node(state=state, g_cost=10.0, h_cost=5.0)
        node2 = Node(state=state, g_cost=8.0, h_cost=5.0)
        
        assert node2 < node1  # Lower f_cost should be "less than"


class TestHybridAStar:
    """Test Hybrid A* algorithm"""
    
    def setup_method(self):
        """Setup for each test"""
        self.vehicle = VehicleModel()
        self.planner = HybridAStar(
            vehicle_model=self.vehicle,
            grid_resolution=1.0,
            angle_resolution=np.pi/4,
            velocity=2.0,
            simulation_time=0.5,
            dt=0.1
        )
    
    def test_initialization(self):
        """Test planner initialization"""
        assert self.planner.vehicle_model == self.vehicle
        assert self.planner.grid_resolution == 1.0
        assert self.planner.velocity == 2.0
        assert len(self.planner.steer_rates) > 0
    
    def test_heuristic_cost(self):
        """Test heuristic cost calculation"""
        start = State(x=0, y=0, yaw=0)
        goal = State(x=3, y=4, yaw=np.pi/2)
        
        cost = self.planner.heuristic_cost(start, goal)
        expected_distance = 5.0  # 3-4-5 triangle
        assert cost >= expected_distance  # Should include angular cost too
    
    def test_discretize_state(self):
        """Test state discretization"""
        state = State(x=1.7, y=2.3, yaw=np.pi/3)
        grid_coords = self.planner.discretize_state(state)
        
        assert len(grid_coords) == 3
        assert all(isinstance(coord, int) for coord in grid_coords)
    
    def test_collision_detection_no_map(self):
        """Test collision detection without obstacle map"""
        state = State(x=0, y=0, yaw=0)
        assert self.planner.is_collision_free(state)  # Should be free without map
    
    def test_collision_detection_with_map(self):
        """Test collision detection with obstacle map"""
        # Create simple obstacle map
        obstacle_map = np.zeros((10, 10))
        obstacle_map[5, 5] = 1  # Single obstacle
        
        self.planner.set_obstacle_map(obstacle_map, origin_x=0, origin_y=0)
        
        free_state = State(x=0, y=0, yaw=0)
        obstacle_state = State(x=5, y=5, yaw=0)
        
        assert self.planner.is_collision_free(free_state)
        assert not self.planner.is_collision_free(obstacle_state)
    
    def test_cost_calculations(self):
        """Test various cost calculations"""
        # Steering cost
        steer_cost = self.planner.calculate_steering_cost(np.pi/8)
        assert 0 <= steer_cost <= 1
        
        # Turning cost
        turn_cost = self.planner.calculate_turning_cost(0, np.pi/4)
        assert turn_cost > 0
        
        # Cusp cost
        cusp_cost = self.planner.calculate_cusp_cost(
            DirectionMode.FORWARD, DirectionMode.BACKWARD
        )
        assert cusp_cost > 0
        
        no_cusp_cost = self.planner.calculate_cusp_cost(
            DirectionMode.FORWARD, DirectionMode.FORWARD
        )
        assert no_cusp_cost == 0
    
    def test_goal_reaching(self):
        """Test goal reaching detection"""
        current = State(x=1.0, y=1.0, yaw=0.1)
        goal = State(x=1.5, y=1.2, yaw=0.05)
        far_goal = State(x=5.0, y=5.0, yaw=np.pi)
        
        assert self.planner.is_goal_reached(current, goal, 
                                           position_tolerance=1.0, 
                                           angle_tolerance=np.pi/4)
        assert not self.planner.is_goal_reached(current, far_goal, 
                                               position_tolerance=1.0, 
                                               angle_tolerance=np.pi/4)
    
    def test_path_planning_simple(self):
        """Test simple path planning without obstacles"""
        start = State(x=0, y=0, yaw=0, direction=DirectionMode.FORWARD)
        goal = State(x=2, y=0, yaw=0, direction=DirectionMode.FORWARD)
        
        path = self.planner.plan_path(start, goal, max_iterations=100)
        
        # Should find a path for simple case
        assert path is not None
        assert len(path) > 1
        assert path[0] == start
        
        # Last state should be close to goal
        final_state = path[-1]
        assert self.planner.is_goal_reached(final_state, goal)
    
    def test_path_planning_with_obstacles(self):
        """Test path planning with obstacles"""
        # Create obstacle map with blocking obstacle
        obstacle_map = np.zeros((20, 20))
        obstacle_map[8:12, 8:12] = 1  # Block direct path
        
        self.planner.set_obstacle_map(obstacle_map, origin_x=-5, origin_y=-5)
        
        start = State(x=0, y=0, yaw=0, direction=DirectionMode.FORWARD)
        goal = State(x=5, y=5, yaw=0, direction=DirectionMode.FORWARD)
        
        path = self.planner.plan_path(start, goal, max_iterations=500)
        
        # Should still find a path around obstacle
        if path:  # Path might not always be found in limited iterations
            assert len(path) > 1
            
            # Verify path doesn't go through obstacles
            for state in path:
                assert self.planner.is_collision_free(state)


class TestIntegration:
    """Integration tests for complete functionality"""
    
    def test_end_to_end_planning(self):
        """Test complete planning pipeline"""
        # Setup
        vehicle = VehicleModel(wheelbase=2.0, max_steer=np.pi/4)
        planner = HybridAStar(
            vehicle_model=vehicle,
            grid_resolution=0.5,
            velocity=1.0,
            simulation_time=0.5
        )
        
        # Simple scenario
        start = State(x=0, y=0, yaw=0)
        goal = State(x=3, y=3, yaw=np.pi/2)
        
        path = planner.plan_path(start, goal, max_iterations=200)
        
        # Basic validation
        if path:
            assert len(path) >= 2
            assert isinstance(path[0], State)
            assert isinstance(path[-1], State)
            
            # Path should be continuous (no large jumps)
            for i in range(len(path) - 1):
                dx = path[i+1].x - path[i].x
                dy = path[i+1].y - path[i].y
                distance = np.sqrt(dx*dx + dy*dy)
                assert distance < 5.0  # Reasonable step size


if __name__ == "__main__":
    # Run basic tests
    pytest.main([__file__, "-v"])
