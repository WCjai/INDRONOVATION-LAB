import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
from scipy.spatial import distance

def generate_coverage_path(points, line_spacing):
    # Create a Path object from the given points
    path = Path(points)

    # Calculate the bounding box of the points
    min_x, min_y = np.min(points, axis=0)
    max_x, max_y = np.max(points, axis=0)

    # Generate points within the bounding box
    x = np.arange(min_x, max_x + line_spacing, line_spacing)
    y = np.arange(min_y, max_y + line_spacing, line_spacing)
    xv, yv = np.meshgrid(x, y)
    points_within_box = np.column_stack((xv.ravel(), yv.ravel()))

    # Select points inside the polygon
    points_within_polygon = points_within_box[path.contains_points(points_within_box)]

    # Generate the coverage path using the Nearest Neighbor algorithm
    path_points = [tuple(points_within_polygon[0])]  # Starting point
    remaining_points = set(tuple(point) for point in points_within_polygon[1:])

    while remaining_points:
        last_point = path_points[-1]
        nearest_point = min(remaining_points, key=lambda p: distance.euclidean(p, last_point))
        path_points.append(nearest_point)
        remaining_points.remove(nearest_point)

    return path_points

# Define the corner points of the odd-shaped area
points = [(1, 1), (2, 4), (3, 2), (1, 1), (3, 2)]

# Parameters
line_spacing = 90  # Spacing between the lines

# Generate the coverage path within the polygon shape
coverage_path = generate_coverage_path(points, line_spacing)

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
