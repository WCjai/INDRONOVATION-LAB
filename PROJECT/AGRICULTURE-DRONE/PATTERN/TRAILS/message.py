import numpy as np
import matplotlib.pyplot as plt
from math import radians, sin, cos, sqrt, atan2

# Define the latitude and longitude points (4 points for a square)
latitude = [40.7128, 40.7128, 40.7228, 40.7228]  # Sample latitude values
longitude = [-74.0060, -74.0160, -74.0160, -74.0060]  # Sample longitude values

# Define pattern parameters
width_between_lines = 10  # Width between lines in meters
num_lines = 5  # Number of lines
width_from_border = 10  # Width from the border in meters

# Convert latitude and longitude to meters
def lat_lon_to_meters(lat1, lon1, lat2, lon2):
    R = 6371000  # Radius of the Earth in meters

    lat1_rad = radians(lat1)
    lon1_rad = radians(lon1)
    lat2_rad = radians(lat2)
    lon2_rad = radians(lon2)

    dlat = lat2_rad - lat1_rad
    dlon = lon2_rad - lon1_rad

    a = sin(dlat / 2) ** 2 + cos(lat1_rad) * cos(lat2_rad) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = R * c

    return distance

# Calculate the bounding box of the given latitude and longitude points
min_latitude = np.min(latitude)
max_latitude = np.max(latitude)
min_longitude = np.min(longitude)
max_longitude = np.max(longitude)

# Calculate the dimensions of the area in meters
width = lat_lon_to_meters(min_latitude, min_longitude, min_latitude, max_longitude)
height = lat_lon_to_meters(min_latitude, min_longitude, max_latitude, min_longitude)

# Calculate the spacing between lines
line_spacing = (height - 2 * width_from_border) / (num_lines - 1)

# Generate waypoints within the defined area
waypoints = []
for i in range(num_lines):
    y = min_latitude + (width_from_border + i * line_spacing) * 360 / (2 * np.pi * 6371000)

    # Generate waypoints along each line with the desired width between lines
    for j, x in enumerate(np.arange(min_longitude + width_from_border, max_longitude - width_from_border, width_between_lines)):
        # Alternate the direction of each line to create a back-and-forth pattern
        if i % 2 == 0:
            waypoints.append((x, y))
        else:
            waypoints.append((max_longitude - (x - min_longitude), y))

# Print the generated waypoints
print (waypoints)
for waypoint in waypoints:
    print("Latitude:", waypoint[0], "Longitude:", waypoint[1])

# Visualize the generated waypoints on a map
plt.scatter(longitude, latitude, color='red', label='Boundary')
plt.scatter([point[0] for point in waypoints], [point[1] for point in waypoints],
            color='blue', label='Waypoints')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.legend()
plt.title('Generated Waypoints')
plt.show()
