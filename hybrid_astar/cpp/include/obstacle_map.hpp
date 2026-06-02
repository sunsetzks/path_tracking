/**
 * @file obstacle_map.hpp
 * @brief Obstacle map utilities for path planning
 */

#pragma once

#include <vector>
#include <string>

namespace hybrid_astar {

/**
 * @brief Obstacle map utilities
 */
class ObstacleMap {
public:
    /**
     * @brief Create a simple test obstacle map
     * @param width Map width in cells
     * @param height Map height in cells
     * @return 2D obstacle map (0=free, 1=occupied)
     */
    static std::vector<std::vector<int>> create_test_map(int width, int height);
    
    /**
     * @brief Create obstacle map with rectangular obstacles
     * @param width Map width in cells
     * @param height Map height in cells
     * @param obstacles Vector of rectangles (x, y, width, height)
     * @return 2D obstacle map
     */
    static std::vector<std::vector<int>> create_map_with_rectangles(
        int width, int height,
        const std::vector<std::vector<int>>& obstacles);
    
    /**
     * @brief Load obstacle map from file
     * @param filename Path to map file
     * @return 2D obstacle map
     */
    static std::vector<std::vector<int>> load_from_file(const std::string& filename);
    
    /**
     * @brief Save obstacle map to file
     * @param obstacle_map Map to save
     * @param filename Output filename
     * @return Success status
     */
    static bool save_to_file(const std::vector<std::vector<int>>& obstacle_map,
                            const std::string& filename);
    
    /**
     * @brief Print obstacle map to console
     * @param obstacle_map Map to print
     */
    static void print_map(const std::vector<std::vector<int>>& obstacle_map);
    
    /**
     * @brief Add border obstacles to map
     * @param obstacle_map Map to modify
     * @param border_width Border width in cells
     */
    static void add_border(std::vector<std::vector<int>>& obstacle_map, int border_width = 1);
    
    /**
     * @brief Check if coordinates are within map bounds
     * @param obstacle_map Map to check
     * @param x X coordinate
     * @param y Y coordinate
     * @return True if within bounds
     */
    static bool is_valid_coordinates(const std::vector<std::vector<int>>& obstacle_map,
                                   int x, int y);
    
    /**
     * @brief Get map dimensions
     * @param obstacle_map Input map
     * @return Pair of (width, height)
     */
    static std::pair<int, int> get_dimensions(const std::vector<std::vector<int>>& obstacle_map);
};

} // namespace hybrid_astar
