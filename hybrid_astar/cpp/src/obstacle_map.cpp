/**
 * @file obstacle_map.cpp
 * @brief Implementation of obstacle map utilities
 */

#include "obstacle_map.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>

namespace hybrid_astar {

std::vector<std::vector<int>> ObstacleMap::create_test_map(int width, int height) {
    std::vector<std::vector<int>> map(height, std::vector<int>(width, 0));
    
    // Add some test obstacles
    // Vertical wall
    for (int y = height/4; y < 3*height/4; ++y) {
        map[y][width/3] = 1;
    }
    
    // Horizontal wall
    for (int x = 0; x < 2*width/3; ++x) {
        map[height/2][x] = 1;
    }
    
    // Add border
    add_border(map, 1);
    
    return map;
}

std::vector<std::vector<int>> ObstacleMap::create_map_with_rectangles(
    int width, int height,
    const std::vector<std::vector<int>>& obstacles) {
    
    std::vector<std::vector<int>> map(height, std::vector<int>(width, 0));
    
    // Add rectangular obstacles
    for (const auto& rect : obstacles) {
        if (rect.size() >= 4) {
            int x = rect[0];
            int y = rect[1];
            int w = rect[2];
            int h = rect[3];
            
            for (int i = y; i < y + h && i < height; ++i) {
                for (int j = x; j < x + w && j < width; ++j) {
                    if (i >= 0 && j >= 0) {
                        map[i][j] = 1;
                    }
                }
            }
        }
    }
    
    return map;
}

std::vector<std::vector<int>> ObstacleMap::load_from_file(const std::string& filename) {
    std::vector<std::vector<int>> map;
    std::ifstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << std::endl;
        return map;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        std::vector<int> row;
        for (char c : line) {
            if (c == '0' || c == '1') {
                row.push_back(c - '0');
            }
        }
        if (!row.empty()) {
            map.push_back(row);
        }
    }
    
    file.close();
    return map;
}

bool ObstacleMap::save_to_file(const std::vector<std::vector<int>>& obstacle_map,
                              const std::string& filename) {
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        std::cerr << "Error: Could not create file " << filename << std::endl;
        return false;
    }
    
    for (const auto& row : obstacle_map) {
        for (int cell : row) {
            file << cell;
        }
        file << '\n';
    }
    
    file.close();
    return true;
}

void ObstacleMap::print_map(const std::vector<std::vector<int>>& obstacle_map) {
    for (const auto& row : obstacle_map) {
        for (int cell : row) {
            std::cout << (cell ? '#' : '.');
        }
        std::cout << '\n';
    }
}

void ObstacleMap::add_border(std::vector<std::vector<int>>& obstacle_map, int border_width) {
    if (obstacle_map.empty()) return;
    
    int height = static_cast<int>(obstacle_map.size());
    int width = static_cast<int>(obstacle_map[0].size());
    
    // Top and bottom borders
    for (int y = 0; y < border_width; ++y) {
        for (int x = 0; x < width; ++x) {
            obstacle_map[y][x] = 1;  // Top border
            obstacle_map[height - 1 - y][x] = 1;  // Bottom border
        }
    }
    
    // Left and right borders
    for (int x = 0; x < border_width; ++x) {
        for (int y = 0; y < height; ++y) {
            obstacle_map[y][x] = 1;  // Left border
            obstacle_map[y][width - 1 - x] = 1;  // Right border
        }
    }
}

bool ObstacleMap::is_valid_coordinates(const std::vector<std::vector<int>>& obstacle_map,
                                     int x, int y) {
    if (obstacle_map.empty()) return false;
    
    int height = static_cast<int>(obstacle_map.size());
    int width = static_cast<int>(obstacle_map[0].size());
    
    return (x >= 0 && x < width && y >= 0 && y < height);
}

std::pair<int, int> ObstacleMap::get_dimensions(const std::vector<std::vector<int>>& obstacle_map) {
    if (obstacle_map.empty()) {
        return {0, 0};
    }
    
    int height = static_cast<int>(obstacle_map.size());
    int width = static_cast<int>(obstacle_map[0].size());
    
    return {width, height};
}

} // namespace hybrid_astar
