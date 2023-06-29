import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString

def generate_coverage_path(points, num_lines, line_spacing, turn_spacing):
    # Create a Shapely Polygon object from the given points
    polygon = Polygon(points)

    # Calculate the bounding box of the points
    min_x, min_y = np.min(points, axis=0)
    max_x, max_y = np.max(points, axis=0)

    # Calculate the size of each cell
    cell_size = np.sqrt((max_x - min_x) * (max_y - min_y) / num_lines)
    line_spacing = line_spacing * cell_size
    turn_spacing = turn_spacing * cell_size

    # Generate the coverage path with snake-like pattern
    path_points = []
    for line_idx in range(num_lines):
        y = min_y + line_idx * (cell_size + line_spacing) + cell_size / 2

        # Check if the line needs to turn upwards or downwards
        if line_idx % 2 == 0:
            # Move right with turn spacing
            path_points.append((min_x, y))
            path_points.append((min_x + turn_spacing, y))

            # Move up
            path_points.append((min_x + turn_spacing, y + cell_size))
            path_points.append((min_x + turn_spacing, y + cell_size + turn_spacing))

            # Move left with turn spacing
            path_points.append((min_x, y + cell_size + turn_spacing))
            path_points.append((min_x - turn_spacing, y + cell_size + turn_spacing))
        else:
            # Move left with turn spacing
            path_points.append((max_x, y))
            path_points.append((max_x - turn_spacing, y))

            # Move up
            path_points.append((max_x - turn_spacing, y + cell_size))
            path_points.append((max_x - turn_spacing, y + cell_size + turn_spacing))

            # Move right with turn spacing
            path_points.append((max_x, y + cell_size + turn_spacing))
            path_points.append((max_x + turn_spacing, y + cell_size + turn_spacing))

    return path_points

# Define the corner points of the odd-shaped area
points = [(1, 4), (3, 4), (3, 1), (1, 1), (1, 4)]

# Parameters
num_lines = 10  # Number of lines
line_spacing = 0.09  # Spacing between the lines
turn_spacing = 0  # Spacing in the turns

# Generate the coverage path within the polygon shape
coverage_path = generate_coverage_path(points, num_lines, line_spacing, turn_spacing)

# Plotting the coverage path and shape
path_x = [point[0] for point in coverage_path]
path_y = [point[1] for point in coverage_path]

shape_x = [point[0] for point in points]
shape_y = [point[1] for point in points]

plt.plot(path_x, path_y, 'b-')
plt.plot(shape_x, shape_y, 'ro-')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('UAV Coverage Path Planning')
plt.grid(True)
plt.show()
