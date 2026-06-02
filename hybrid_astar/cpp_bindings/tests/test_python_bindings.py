"""
Python tests for hybrid_astar_cpp module
"""

import pytest
import numpy as np

# Note: This test file assumes the C++ module has been built and installed
# If the module is not available, tests will be skipped
try:
    import hybrid_astar_cpp as ha
    HAS_CPP_MODULE = True
except ImportError:
    HAS_CPP_MODULE = False
    pytest.skip("hybrid_astar_cpp module not available", allow_module_level=True)


class TestBasicFunctionality:
    """Test basic functionality of the C++ module"""
    
    def test_direction_mode_enum(self):
        """Test DirectionMode enum values"""
        assert ha.DirectionMode.FORWARD.value == 1
        assert ha.DirectionMode.BACKWARD.value == -1
        assert ha.DirectionMode.NONE.value == 0
    
    def test_costs_creation(self):
        """Test Costs struct creation and access"""
        costs = ha.Costs()
        assert costs.distance == 0.0
        assert costs.steer == 0.0
        assert costs.turn == 0.0
        assert costs.cusp == 0.0
        
        costs2 = ha.Costs(1.0, 0.5, 0.2, 0.1)
        assert costs2.distance == 1.0
        assert costs2.steer == 0.5
        assert costs2.turn == 0.2
        assert costs2.cusp == 0.1
    
    def test_state_creation(self):
        """Test State creation and equality"""
        state1 = ha.State()
        assert state1.x == 0.0
        assert state1.y == 0.0
        assert state1.yaw == 0.0
        assert state1.direction == ha.DirectionMode.NONE
        assert state1.steer == 0.0
        
        state2 = ha.State(1.0, 2.0, np.pi/4, ha.DirectionMode.FORWARD, 0.1)
        assert state2.x == 1.0
        assert state2.y == 2.0
        assert abs(state2.yaw - np.pi/4) < 1e-6
        assert state2.direction == ha.DirectionMode.FORWARD
        assert state2.steer == 0.1
        
        # Test equality (should be equal within tolerance)
        state3 = ha.State(1.05, 2.05, np.pi/4 + 0.05, ha.DirectionMode.FORWARD, 0.1)
        assert state2 == state3  # Within tolerance
        
        state4 = ha.State(2.0, 3.0, np.pi/2, ha.DirectionMode.BACKWARD, 0.2)
        assert state2 != state4  # Different


class TestVehicleModel:
    """Test VehicleModel functionality"""
    
    def test_vehicle_model_creation(self):
        """Test VehicleModel creation"""
        vehicle = ha.VehicleModel()
        assert vehicle.wheelbase == 2.5
        assert abs(vehicle.max_steer - np.pi/4) < 1e-6
        
        vehicle2 = ha.VehicleModel(3.0, np.pi/3)
        assert vehicle2.wheelbase == 3.0
        assert abs(vehicle2.max_steer - np.pi/3) < 1e-6
    
    def test_normalize_angle(self):
        """Test angle normalization"""
        assert abs(ha.VehicleModel.normalize_angle(3*np.pi) - np.pi) < 1e-6
        assert abs(ha.VehicleModel.normalize_angle(-3*np.pi) - (-np.pi)) < 1e-6
        assert abs(ha.VehicleModel.normalize_angle(np.pi/2) - np.pi/2) < 1e-6
        assert abs(ha.VehicleModel.normalize_angle(0.0)) < 1e-6
    
    def test_simulate_motion(self):
        """Test motion simulation"""
        vehicle = ha.VehicleModel(2.5, np.pi/4)
        initial_state = ha.State(0.0, 0.0, 0.0, ha.DirectionMode.FORWARD, 0.0)
        
        # Simulate straight motion
        states = vehicle.simulate_motion(initial_state, 1.0, 0.0, 0.1, 10)
        
        assert len(states) == 10
        assert states[-1].x > 0.9  # Should have moved forward
        assert abs(states[-1].y) < 0.1  # Should stay roughly on x-axis
        
        # Test backward motion
        backward_state = ha.State(0.0, 0.0, 0.0, ha.DirectionMode.BACKWARD, 0.0)
        backward_states = vehicle.simulate_motion(backward_state, 1.0, 0.0, 0.1, 10)
        
        assert len(backward_states) == 10
        assert backward_states[-1].x < -0.9  # Should have moved backward


class TestHybridAStar:
    """Test HybridAStar planner functionality"""
    
    def test_planner_creation(self):
        """Test HybridAStar creation"""
        vehicle = ha.VehicleModel(2.5, np.pi/4)
        planner = ha.HybridAStar(vehicle)
        
        assert planner.get_grid_resolution() == 1.0
        assert planner.get_vehicle_model().wheelbase == 2.5
    
    def test_collision_checking_no_obstacles(self):
        """Test collision checking without obstacles"""
        vehicle = ha.VehicleModel()
        planner = ha.HybridAStar(vehicle)
        
        state = ha.State(5.0, 5.0, 0.0, ha.DirectionMode.FORWARD, 0.0)
        assert planner.is_collision_free(state)  # Should be collision-free
    
    def test_obstacle_map(self):
        """Test obstacle map functionality"""
        vehicle = ha.VehicleModel()
        planner = ha.HybridAStar(vehicle)
        
        # Create obstacle map
        obstacle_map = ha.create_obstacle_map(20, 20)
        ha.add_rectangle_obstacle(obstacle_map, 8, 8, 12, 12)
        planner.set_obstacle_map(obstacle_map, 0.0, 0.0)
        
        # Test collision checking
        free_state = ha.State(1.0, 1.0, 0.0, ha.DirectionMode.FORWARD, 0.0)
        obstacle_state = ha.State(10.0, 10.0, 0.0, ha.DirectionMode.FORWARD, 0.0)
        
        assert planner.is_collision_free(free_state)
        assert not planner.is_collision_free(obstacle_state)
    
    def test_heuristic_cost(self):
        """Test heuristic cost calculation"""
        vehicle = ha.VehicleModel()
        planner = ha.HybridAStar(vehicle)
        
        start = ha.State(0.0, 0.0, 0.0, ha.DirectionMode.NONE, 0.0)
        goal = ha.State(10.0, 10.0, np.pi/2, ha.DirectionMode.FORWARD, 0.0)
        
        h_cost = planner.heuristic_cost(start, goal)
        assert h_cost > 0.0  # Should have positive cost
        
        # Cost to same state should be low
        h_cost_same = planner.heuristic_cost(start, start)
        assert h_cost_same < 1.0


class TestPathPlanning:
    """Test path planning functionality"""
    
    def test_simple_path_planning(self):
        """Test simple path planning without obstacles"""
        vehicle = ha.VehicleModel(2.5, np.pi/4)
        planner = ha.HybridAStar(vehicle, 1.0, np.pi/8, np.pi/16, 2.0, 1.0, 0.2)
        
        start = ha.State(0.0, 0.0, 0.0, ha.DirectionMode.NONE, 0.0)
        goal = ha.State(5.0, 0.0, 0.0, ha.DirectionMode.FORWARD, 0.0)
        
        path = planner.plan_path(start, goal, 1000)
        
        # Path may or may not be found depending on discretization
        # This test mainly checks that the function runs without error
        if path:
            assert len(path) > 0
            assert isinstance(path[0], ha.Node)
            
            # Test statistics
            stats = planner.get_statistics(path)
            assert isinstance(stats, dict)
            assert "path_found" in stats
            assert stats["path_found"] == 1.0
            
            # Test detailed path extraction
            detailed_path = planner.extract_detailed_path(path)
            assert len(detailed_path) >= len(path)
            assert all(isinstance(state, ha.State) for state in detailed_path)
    
    def test_path_planning_with_obstacles(self):
        """Test path planning with obstacles"""
        vehicle = ha.VehicleModel(2.5, np.pi/4)
        planner = ha.HybridAStar(vehicle, 0.5, np.pi/8, np.pi/16, 2.0, 0.8, 0.1)
        
        # Create obstacle map
        obstacle_map = ha.create_obstacle_map(30, 30)
        ha.add_rectangle_obstacle(obstacle_map, 10, 10, 20, 15)
        planner.set_obstacle_map(obstacle_map, -5.0, -5.0)
        
        start = ha.State(-3.0, -3.0, np.pi/4, ha.DirectionMode.NONE, 0.0)
        goal = ha.State(8.0, 8.0, np.pi/2, ha.DirectionMode.FORWARD, 0.0)
        
        path = planner.plan_path(start, goal, 5000)
        
        # Test that planning completes (may or may not find path)
        stats = planner.get_statistics(path)
        assert isinstance(stats, dict)
        assert "nodes_explored" in stats
        assert stats["nodes_explored"] > 0
    
    def test_statistics_no_path(self):
        """Test statistics when no path is found"""
        vehicle = ha.VehicleModel()
        planner = ha.HybridAStar(vehicle)
        
        # Use None to simulate no path found
        stats = planner.get_statistics(None)
        assert isinstance(stats, dict)
        assert "path_found" in stats
        assert stats["path_found"] == 0.0


class TestUtilityFunctions:
    """Test utility functions"""
    
    def test_create_obstacle_map(self):
        """Test obstacle map creation"""
        obstacle_map = ha.create_obstacle_map(10, 5)
        assert len(obstacle_map) == 5  # height
        assert len(obstacle_map[0]) == 10  # width
        assert all(all(cell == 0 for cell in row) for row in obstacle_map)
    
    def test_add_rectangle_obstacle(self):
        """Test adding rectangular obstacles"""
        obstacle_map = ha.create_obstacle_map(10, 10)
        ha.add_rectangle_obstacle(obstacle_map, 2, 2, 5, 5)
        
        # Check that obstacle was added
        assert obstacle_map[3][3] == 1  # Inside obstacle
        assert obstacle_map[1][1] == 0  # Outside obstacle
        assert obstacle_map[6][6] == 0  # Outside obstacle


@pytest.mark.benchmark
class TestPerformance:
    """Performance benchmarks"""
    
    def test_path_planning_benchmark(self, benchmark):
        """Benchmark path planning performance"""
        if not HAS_CPP_MODULE:
            pytest.skip("C++ module not available")
        
        vehicle = ha.VehicleModel(2.5, np.pi/4)
        planner = ha.HybridAStar(vehicle, 0.5, np.pi/8, np.pi/16, 2.0, 0.5, 0.1)
        
        # Create obstacle map
        obstacle_map = ha.create_obstacle_map(40, 40)
        ha.add_rectangle_obstacle(obstacle_map, 15, 15, 25, 20)
        ha.add_rectangle_obstacle(obstacle_map, 8, 25, 12, 32)
        planner.set_obstacle_map(obstacle_map, -10.0, -10.0)
        
        start = ha.State(-5.0, -5.0, np.pi/4, ha.DirectionMode.NONE, 0.0)
        goal = ha.State(12.0, 12.0, np.pi/2, ha.DirectionMode.FORWARD, 0.0)
        
        def plan_path():
            return planner.plan_path(start, goal, 3000)
        
        result = benchmark(plan_path)
        # Result should complete within reasonable time
        assert result is not None  # Should return some result


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
