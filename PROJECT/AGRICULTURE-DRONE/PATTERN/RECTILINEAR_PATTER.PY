import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point

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
        x = min_x + line_idx * cell_width + cell_width / 2
        if line_idx % 2 == 0:
            for point_idx in range(num_lines):
                y = min_y + point_idx * cell_height + cell_height / 2
                if polygon.contains(Point(x, y)):
                    path_points.append((x, y))
        else:
            for point_idx in reversed(range(num_lines)):
                y = min_y + point_idx * cell_height + cell_height / 2
                if polygon.contains(Point(x, y)):
                    path_points.append((x, y))

    return path_points

# Define the corner points of the odd-shaped area
points = [(1, 3), (3, 5), (6, 8), (3, 10), (1,10), (1,3)]

# Parameters
num_lines = 60 # Number of lines

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
