#!/usr/bin/env python3
import hybrid_astar_cpp as ha

print("Testing obstacle map...")

# Create obstacle map
obstacle_map = ha.create_obstacle_map(10, 10)
print(f"Initial map size: {len(obstacle_map)} x {len(obstacle_map[0])}")

# Print initial map
print("Initial map:")
for i, row in enumerate(obstacle_map):
    print(f"  Row {i}: {''.join(str(cell) for cell in row)}")

# Add rectangle obstacle
print("\nAdding rectangle obstacle from (2,2) to (5,5)...")
obstacle_map = ha.add_rectangle_obstacle(obstacle_map, 2, 2, 5, 5)

# Print map after adding obstacle
print("Map after adding obstacle:")
for i, row in enumerate(obstacle_map):
    print(f"  Row {i}: {''.join(str(cell) for cell in row)}")

# Count obstacles
obstacle_count = sum(sum(row) for row in obstacle_map)
print(f"\nTotal obstacle cells: {obstacle_count}")
