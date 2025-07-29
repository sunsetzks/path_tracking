/**
 * @file pybind_module.cpp
 * @brief Python bindings for Hybrid A* algorithm using pybind11
 * 
 * @author Converted from Python implementation
 * @date 2025-07-29
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include "hybrid_astar.hpp"

namespace py = pybind11;
using namespace hybrid_astar;
using namespace pybind11::literals;

PYBIND11_MODULE(hybrid_astar_cpp, m) {
    m.doc() = "Hybrid A* Path Planning Algorithm - C++ implementation with Python bindings";
    
    // DirectionMode enum
    py::enum_<DirectionMode>(m, "DirectionMode")
        .value("FORWARD", DirectionMode::FORWARD)
        .value("BACKWARD", DirectionMode::BACKWARD)
        .value("NONE", DirectionMode::NONE)
        .export_values();
    
    // Costs struct
    py::class_<Costs>(m, "Costs")
        .def(py::init<>())
        .def(py::init<double, double, double, double>(),
             "distance"_a, "steer"_a, "turn"_a, "cusp"_a)
        .def_readwrite("distance", &Costs::distance)
        .def_readwrite("steer", &Costs::steer)
        .def_readwrite("turn", &Costs::turn)
        .def_readwrite("cusp", &Costs::cusp)
        .def("__repr__", [](const Costs& c) {
            return "Costs(distance=" + std::to_string(c.distance) +
                   ", steer=" + std::to_string(c.steer) +
                   ", turn=" + std::to_string(c.turn) +
                   ", cusp=" + std::to_string(c.cusp) + ")";
        });
    
    // State struct
    py::class_<State>(m, "State")
        .def(py::init<>())
        .def(py::init<double, double, double, DirectionMode, double>(),
             "x"_a, "y"_a, "yaw"_a, "direction"_a = DirectionMode::NONE, "steer"_a = 0.0)
        .def_readwrite("x", &State::x)
        .def_readwrite("y", &State::y)
        .def_readwrite("yaw", &State::yaw)
        .def_readwrite("direction", &State::direction)
        .def_readwrite("steer", &State::steer)
        .def(py::self == py::self)
        .def(py::self != py::self)
        .def("__repr__", [](const State& s) {
            return "State(x=" + std::to_string(s.x) +
                   ", y=" + std::to_string(s.y) +
                   ", yaw=" + std::to_string(s.yaw) +
                   ", direction=" + std::to_string(static_cast<int>(s.direction)) +
                   ", steer=" + std::to_string(s.steer) + ")";
        });
    
    // Node class
    py::class_<Node, std::shared_ptr<Node>>(m, "Node")
        .def(py::init<>())
        .def(py::init<const State&>(), "state"_a)
        .def(py::init<const State&, double, std::shared_ptr<Node>>(),
             "state"_a, "g_cost"_a, "parent"_a = nullptr)
        .def_readwrite("state", &Node::state)
        .def_readwrite("g_cost", &Node::g_cost)
        .def_readwrite("h_cost", &Node::h_cost)
        .def_readwrite("parent", &Node::parent)
        .def_readwrite("costs", &Node::costs)
        .def_readwrite("forward_simulation_trajectory", &Node::forward_simulation_trajectory)
        .def("f_cost", &Node::f_cost)
        .def("steer_cost", &Node::steer_cost)
        .def("turn_cost", &Node::turn_cost)
        .def("cusp_cost", &Node::cusp_cost)
        .def("__repr__", [](const Node& n) {
            return "Node(state=" + std::to_string(n.state.x) + "," + std::to_string(n.state.y) +
                   ", g_cost=" + std::to_string(n.g_cost) +
                   ", h_cost=" + std::to_string(n.h_cost) + ")";
        });
    
    // VehicleModel class
    py::class_<VehicleModel>(m, "VehicleModel")
        .def(py::init<double, double>(), "wheelbase"_a = 2.5, "max_steer"_a = M_PI/4)
        .def("simulate_motion", &VehicleModel::simulate_motion,
             "state"_a, "velocity"_a, "steer_rate"_a, "dt"_a, "steps"_a = 1,
             "Forward simulate vehicle motion with fixed linear velocity and steering rate")
        .def_static("normalize_angle", &VehicleModel::normalize_angle,
                   "angle"_a, "Normalize angle to [-pi, pi]")
        .def_property_readonly("wheelbase", &VehicleModel::wheelbase)
        .def_property_readonly("max_steer", &VehicleModel::max_steer)
        .def("__repr__", [](const VehicleModel& vm) {
            return "VehicleModel(wheelbase=" + std::to_string(vm.wheelbase()) +
                   ", max_steer=" + std::to_string(vm.max_steer()) + ")";
        });
    
    // HybridAStar class
    py::class_<HybridAStar>(m, "HybridAStar")
        .def(py::init<const VehicleModel&, double, double, double, double, double, double>(),
             "vehicle_model"_a,
             "grid_resolution"_a = 1.0,
             "angle_resolution"_a = M_PI/8,
             "steer_resolution"_a = M_PI/16,
             "velocity"_a = 2.0,
             "simulation_time"_a = 1.0,
             "dt"_a = 0.1)
        .def("set_obstacle_map", &HybridAStar::set_obstacle_map,
             "obstacle_map"_a, "origin_x"_a = 0.0, "origin_y"_a = 0.0,
             "Set obstacle map")
        .def("is_collision_free", &HybridAStar::is_collision_free,
             "state"_a, "Check if state is collision-free")
        .def("heuristic_cost", &HybridAStar::heuristic_cost,
             "state"_a, "goal"_a, "Calculate heuristic cost")
        .def("plan_path", &HybridAStar::plan_path,
             "start"_a, "goal"_a, "max_iterations"_a = 10000,
             "Plan path using Hybrid A* algorithm")
        .def("extract_detailed_path", &HybridAStar::extract_detailed_path,
             "path_nodes"_a, "Extract detailed path states using simulation trajectories")
        .def("get_statistics", &HybridAStar::get_statistics,
             "path"_a, "Get path and search statistics")
        .def("get_explored_nodes", &HybridAStar::get_explored_nodes,
             py::return_value_policy::reference_internal)
        .def("get_simulation_trajectories", &HybridAStar::get_simulation_trajectories,
             py::return_value_policy::reference_internal)
        .def("get_obstacle_map", &HybridAStar::get_obstacle_map,
             py::return_value_policy::reference_internal)
        .def("get_map_origin_x", &HybridAStar::get_map_origin_x)
        .def("get_map_origin_y", &HybridAStar::get_map_origin_y)
        .def("get_grid_resolution", &HybridAStar::get_grid_resolution)
        .def("get_vehicle_model", &HybridAStar::get_vehicle_model,
             py::return_value_policy::reference_internal)
        .def("__repr__", [](const HybridAStar& ha) {
            return "HybridAStar(grid_resolution=" + std::to_string(ha.get_grid_resolution()) + ")";
        });
    
    // Utility functions
    m.def("create_obstacle_map", [](int width, int height) {
        return std::vector<std::vector<int>>(height, std::vector<int>(width, 0));
    }, "width"_a, "height"_a, "Create empty obstacle map");
    
    m.def("add_rectangle_obstacle", [](std::vector<std::vector<int>>& obstacle_map,
                                      int start_x, int start_y, int end_x, int end_y) {
        for (int y = start_y; y < end_y && y < static_cast<int>(obstacle_map.size()); ++y) {
            for (int x = start_x; x < end_x && x < static_cast<int>(obstacle_map[0].size()); ++x) {
                if (y >= 0 && x >= 0) {
                    obstacle_map[y][x] = 1;
                }
            }
        }
        return obstacle_map;  // Return the modified map
    }, "obstacle_map"_a, "start_x"_a, "start_y"_a, "end_x"_a, "end_y"_a,
       "Add rectangular obstacle to map");
    
    // Constants
    m.attr("M_PI") = M_PI;
    
    // Version info
    m.attr("__version__") = "1.0.0";
}
