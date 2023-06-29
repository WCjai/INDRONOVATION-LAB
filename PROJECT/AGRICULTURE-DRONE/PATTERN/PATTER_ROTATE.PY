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
    path_points = []
    for line_idx in range(num_lines):
        x = min_x + line_idx * cell_width + cell_width / 2
        if line_idx % 2 == 0:
            for point_idx in range(num_lines):
                y = min_y + point_idx * cell_height + cell_height / 2
                path_points.append((x, y))
        else:
            for point_idx in reversed(range(num_lines)):
                y = min_y + point_idx * cell_height + cell_height / 2
                path_points.append((x, y))

    # Rotate the coverage path within the boundary shape
    rotated_path_points = []
    for point in path_points:
        rotated_x, rotated_y = rotate_point(point[0], point[1], (min_x + max_x) / 2, (min_y + max_y) / 2, rotation_angle)
        if polygon.contains(Point(rotated_x, rotated_y)):
            rotated_path_points.append((rotated_x, rotated_y))

    return rotated_path_points

def rotate_point(x, y, center_x, center_y, angle):
    # Translate the point to the origin
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
    print ("({},{},30),".format(final_x,final_y))
    return final_x, final_y

# Define the corner points of the odd-shaped area
points = [(-35.36355404, 149.16072123), (-35.35997208, 149.16074877), (-35.35996086, 149.16737159), (-35.36535057, 149.16844556), (-35.36355404, 149.16072123)]

# Parameters
num_lines = 40  # Number of lines
rotation_angle = np.radians(0)  # Rotation angle in degrees
line_spacing = 1 # Line spacing adjustment factor

# Generate the coverage path within the polygon shape
coverage_path = generate_coverage_path(points, num_lines, rotation_angle, line_spacing)
print(len(coverage_path))


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
