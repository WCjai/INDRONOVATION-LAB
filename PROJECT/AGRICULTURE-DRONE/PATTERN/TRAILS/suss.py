import numpy as np
import matplotlib.pyplot as plt

def generate_coverage_path(width, height, line_spacing):
    path = []

    # Start from the bottom-left corner
    current_position = np.array([0, 0])
    path.append(current_position.copy())

    # Move horizontally along each row, alternating direction
    for row in range(1, int(height / line_spacing) + 1):
        current_position[0] += width if row % 2 == 0 else -width
        path.append(current_position.copy())

        # Move to the next row
        current_position[1] += line_spacing
        path.append(current_position.copy())

    return path

# Parameters
width = 10  # Width of the area to be covered
height = 10  # Height of the area to be covered
line_spacing = 1  # Spacing between the lines

# Generate the coverage path
coverage_path = generate_coverage_path(width, height, line_spacing)

# Plotting the coverage path
path_x = [point[0] for point in coverage_path]
path_y = [point[1] for point in coverage_path]

plt.plot(path_x, path_y, 'b-')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('UAV Coverage Path Planning')
plt.grid(True)
plt.show()