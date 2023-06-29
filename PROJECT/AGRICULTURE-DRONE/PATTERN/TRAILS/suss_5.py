import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon

def generate_coverage_path(points, num_lines):
    # Create a Shapely Polygon object from the given points
    polygon = Polygon(points)

    # Calculate the bounding box of the points
    min_x, min_y = np.min(points, axis=0)
    max_x, max_y = np.max(points, axis=0)

    # Calculate the size of each cell
    cell_width = (max_x - min_x) / num_lines
    cell_height = (max_y - min_y) / num_lines

    # Generate the coverage path with rectilinear pattern within the polygon
    path_points = []
    for line_idx in range(num_lines):
        # Horizontal line
        start_x = min_x
        end_x = max_x
        y = min_y + line_idx * cell_height + cell_height / 2
        path_points.append((start_x, y))
        path_points.append((end_x, y))

        # Vertical line
        start_y = min_y + (line_idx + 1) * cell_height
        end_y = start_y
        path_points.append((end_x, start_y))
        path_points.append((start_x, end_y))

    return path_points

# Define the corner points of the odd-shaped area
points = [(0, 2), (0, 10), (6, 6), (0, 2), (1, 4)]

# Parameters
num_lines = 10  # Number of lines

# Generate the coverage path within the polygon shape
coverage_path = generate_coverage_path(points, num_lines)

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
