import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point

def generate_coverage_path(points, num_lines, rotation_angle, line_spacing):
    # Create a Shapely Polygon object from the given points
    polygon = Polygon(points)

    # Calculate the bounding box of the points
    min_x, min_y = np.min(points, axis=0)
    max_x, max_y = np.max(points, axis=0)

    # Calculate the size of each cell
    cell_width = (max_x - min_x) / num_lines
    cell_height = (max_y - min_y) / num_lines

    # Adjust the cell size based on the line spacing
    cell_width *= line_spacing
    cell_height *= line_spacing

    # Generate the coverage path with rectilinear pattern within the bounding box
    path_segments = []
    for line_idx in range(num_lines):
        x = min_x + line_idx * cell_width + cell_width / 2
        if line_idx % 2 == 0:
            for point_idx in range(num_lines):
                y = min_y + point_idx * cell_height + cell_height / 2
                path_segments.append((x, y))
        else:
            for point_idx in reversed(range(num_lines)):
                y = min_y + point_idx * cell_height + cell_height / 2
                path_segments.append((x, y))

    # Rotate and split the coverage path within the boundary shape
    rotated_path_segments = []
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    for segment in path_segments:
        rotated_x, rotated_y = rotate_point(segment[0], segment[1], center_x, center_y, rotation_angle)
        if polygon.contains(Point(rotated_x, rotated_y)):
            rotated_path_segments.append((rotated_x, rotated_y))

    return rotated_path_segments

def rotate_point(x, y, center_x, center_y, angle):
    # Translate the point relative to the center of rotation
    translated_x = x - center_x
    translated_y = y - center_y

    # Apply rotation
    cos_theta = np.cos(angle)
    sin_theta = np.sin(angle)
    rotated_x = cos_theta * translated_x - sin_theta * translated_y
    rotated_y = sin_theta * translated_x + cos_theta * translated_y

    # Translate the point back to the original position
    final_x = rotated_x + center_x
    final_y = rotated_y + center_y

    return final_x, final_y

# Define the corner points of the odd-shaped area
points = [(1, 3), (3, 5), (6, 8), (3, 10), (1,10), (1,3)]

# Parameters
num_lines = 10  # Number of lines (adjust for optimization)
rotation_angle = np.radians(0)  # Rotation angle in degrees
line_spacing = 4  # Line spacing adjustment factor

# Generate the coverage path within the polygon shape
coverage_path = generate_coverage_path(points, num_lines, rotation_angle, line_spacing)

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
