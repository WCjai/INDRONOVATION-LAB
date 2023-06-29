import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString

def generate_coverage_path(points, num_lines):
    # Create a Shapely Polygon object from the given points
    polygon = Polygon(points)

    # Calculate the bounding box of the points
    min_x, min_y = np.min(points, axis=0)
    max_x, max_y = np.max(points, axis=0)

    # Calculate the size of each cell
    cell_width = (max_x - min_x) / num_lines
    cell_height = (max_y - min_y) / num_lines

    # Generate the coverage path with alternating lines within the polygon
    path_points = []
    for line_idx in range(num_lines):
        x_start = max_x if line_idx % 2 == 0 else min_x
       # y_start = min_y + line_idx * cell_height + cell_height / 2
        print (line_idx)
        y_start = min_y + line_idx
        y_end = y_start + cell_height if line_idx % 2 == 0 else y_start - cell_height
        line = LineString([(x_start, y_start), (x_start, y_end)])
        intersection = polygon.intersection(line)
        if isinstance(intersection, LineString):
            path_points.extend(list(intersection.coords))

    return path_points

# Define the corner points of the polygon
points = [(0, 0), (10, 0), (10, 10), (0, 10)]

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
